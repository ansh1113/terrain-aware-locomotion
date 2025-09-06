# File: ~/terrain_locomotion_ws/src/terrain_locomotion/terrain_locomotion/perception/terrain_classifier.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from PIL import Image as PILImage
import json

class TerrainCNN(nn.Module):
    """Simple CNN for terrain classification"""
    def __init__(self, num_classes=4):
        super(TerrainCNN, self).__init__()
        self.features = nn.Sequential(
            # First conv block
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
            
            # Second conv block
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
            
            # Third conv block
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
            
            # Fourth conv block
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4))
        )
        
        self.classifier = nn.Sequential(
            nn.Dropout(0.5),
            nn.Linear(256 * 4 * 4, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.5),
            nn.Linear(512, num_classes)
        )
        
    def forward(self, x):
        x = self.features(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x

class TerrainClassifierNode(Node):
    def __init__(self):
        super().__init__('terrain_classifier')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Terrain classes
        self.classes = ['flat', 'slope', 'rubble', 'stairs']
        self.class_to_idx = {cls: idx for idx, cls in enumerate(self.classes)}
        
        # Initialize model
        self.model = TerrainCNN(num_classes=len(self.classes))
        self.model.eval()  # Set to evaluation mode
        
        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        
        # Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.terrain_class_pub = self.create_publisher(
            String,
            '/terrain/class',
            10
        )
        
        self.terrain_confidence_pub = self.create_publisher(
            Float32,
            '/terrain/confidence',
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/terrain/annotated_image',
            10
        )
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('model_path', '')
        
        # Load pretrained model if available
        model_path = self.get_parameter('model_path').value
        if model_path:
            try:
                self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
                self.get_logger().info(f'Loaded model from {model_path}')
            except Exception as e:
                self.get_logger().warn(f'Could not load model: {e}')
                self.get_logger().info('Using random weights for demonstration')
        else:
            self.get_logger().info('No model path provided, using random weights for demonstration')
        
        self.get_logger().info('Terrain Classifier Node initialized')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Classify terrain
            terrain_class, confidence = self.classify_terrain(cv_image)
            
            # Publish results
            class_msg = String()
            class_msg.data = terrain_class
            self.terrain_class_pub.publish(class_msg)
            
            confidence_msg = Float32()
            confidence_msg.data = float(confidence)
            self.terrain_confidence_pub.publish(confidence_msg)
            
            # Create annotated image
            annotated_image = self.create_annotated_image(cv_image, terrain_class, confidence)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            self.annotated_image_pub.publish(annotated_msg)
            
            # Log results
            confidence_threshold = self.get_parameter('confidence_threshold').value
            if confidence < confidence_threshold:
                self.get_logger().warn(f'Low confidence classification: {terrain_class} ({confidence:.2f})')
            else:
                self.get_logger().info(f'Terrain: {terrain_class} (confidence: {confidence:.2f})')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def classify_terrain(self, cv_image):
        """Classify terrain type from image"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Preprocess image
            input_tensor = self.transform(pil_image).unsqueeze(0)
            
            # Run inference
            with torch.no_grad():
                outputs = self.model(input_tensor)
                probabilities = torch.softmax(outputs, dim=1)
                confidence, predicted_idx = torch.max(probabilities, 1)
                
                predicted_class = self.classes[predicted_idx.item()]
                confidence_score = confidence.item()
                
                return predicted_class, confidence_score
                
        except Exception as e:
            self.get_logger().error(f'Error in terrain classification: {str(e)}')
            # Return default classification
            return 'flat', 0.5
    
    def create_annotated_image(self, image, terrain_class, confidence):
        """Create image with terrain classification overlay"""
        annotated = image.copy()
        
        # Define colors for each terrain type
        colors = {
            'flat': (0, 255, 0),      # Green
            'slope': (0, 165, 255),   # Orange  
            'rubble': (0, 0, 255),    # Red
            'stairs': (255, 0, 0)     # Blue
        }
        
        color = colors.get(terrain_class, (128, 128, 128))  # Gray default
        
        # Add text overlay
        text = f'{terrain_class.upper()}: {confidence:.2f}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        thickness = 2
        
        # Get text size for background rectangle
        (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Draw background rectangle
        cv2.rectangle(annotated, (10, 10), (text_width + 20, text_height + 20), color, -1)
        
        # Draw text
        cv2.putText(annotated, text, (15, text_height + 15), font, font_scale, (255, 255, 255), thickness)
        
        # Add confidence bar
        bar_length = int(300 * confidence)
        cv2.rectangle(annotated, (10, text_height + 40), (310, text_height + 60), (128, 128, 128), -1)
        cv2.rectangle(annotated, (10, text_height + 40), (10 + bar_length, text_height + 60), color, -1)
        
        return annotated

def main(args=None):
    rclpy.init(args=args)
    node = TerrainClassifierNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()