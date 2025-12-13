"""Terrain classifier node with CNN-based and feature-based classification."""

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
import os

try:
    from .models.terrain_cnn import TerrainCNN, MobileNetV2Classifier, load_model
except ImportError:
    # Fallback for when running as script
    from terrain_locomotion.perception.models.terrain_cnn import TerrainCNN, MobileNetV2Classifier, load_model

class TerrainClassifierNode(Node):
    """ROS2 node for terrain classification using CNN or feature-based methods.
    
    This node subscribes to camera images and classifies terrain into four
    categories: flat, slope, rubble, and stairs. It can operate in two modes:
    1. CNN mode: Uses trained neural network (when model is available)
    2. Demo mode: Uses feature-based heuristics (no model required)
    """
    
    def __init__(self):
        super().__init__('terrain_classifier')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Terrain classes
        self.classes = ['flat', 'slope', 'rubble', 'stairs']
        self.class_to_idx = {cls: idx for idx, cls in enumerate(self.classes)}
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_type', 'custom')  # 'custom' or 'mobilenet'
        self.declare_parameter('use_demo_mode', True)  # Use feature-based if no model
        
        model_path = self.get_parameter('model_path').value
        model_type = self.get_parameter('model_type').value
        use_demo_mode = self.get_parameter('use_demo_mode').value
        
        # Try to load model
        self.use_cnn = False
        self.model = None
        
        if model_path and os.path.exists(model_path):
            try:
                self.model = load_model(model_path, model_type=model_type, num_classes=len(self.classes))
                self.use_cnn = True
                self.get_logger().info(f'Loaded {model_type} model from {model_path}')
            except Exception as e:
                self.get_logger().warn(f'Could not load model: {e}')
                if not use_demo_mode:
                    raise
        else:
            # Initialize untrained model for demonstration
            if model_type == 'mobilenet':
                self.model = MobileNetV2Classifier(num_classes=len(self.classes), pretrained=False)
            else:
                self.model = TerrainCNN(num_classes=len(self.classes))
            self.model.eval()
            
            if use_demo_mode:
                self.get_logger().info('Running in DEMO MODE with feature-based classification')
            else:
                self.get_logger().warn('No model provided, using random weights')
                self.use_cnn = True
        
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
        """Classify terrain type from image using CNN or feature-based method.
        
        Args:
            cv_image: OpenCV image (BGR format)
            
        Returns:
            tuple: (terrain_class, confidence_score)
        """
        try:
            if self.use_cnn:
                return self._classify_with_cnn(cv_image)
            else:
                return self._classify_with_features(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error in terrain classification: {str(e)}')
            return 'flat', 0.5
    
    def _classify_with_cnn(self, cv_image):
        """Classify using CNN model."""
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
    
    def _classify_with_features(self, cv_image):
        """Classify using hand-crafted features (demo mode without trained model).
        
        This method analyzes image features like color, texture, and edges to
        classify terrain. It's designed to work without a trained model.
        """
        # Resize for consistent feature extraction
        image = cv2.resize(cv_image, (224, 224))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Feature 1: Color variance (rubble has high variance)
        color_variance = np.std(image)
        
        # Feature 2: Texture complexity (edges)
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.sum(edges > 0) / edges.size
        
        # Feature 3: Gradient orientation (slopes have directional gradients)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient_angle = np.arctan2(sobely, sobelx)
        gradient_std = np.std(gradient_angle)
        
        # Feature 4: Vertical edges (stairs have strong vertical patterns)
        vertical_edges = np.sum(np.abs(sobelx) > 50)
        horizontal_edges = np.sum(np.abs(sobely) > 50)
        
        # Feature 5: Color uniformity (flat terrain is more uniform)
        h, s, v = cv2.split(hsv)
        saturation_mean = np.mean(s)
        value_variance = np.std(v)
        
        # Heuristic classification based on features
        scores = {
            'flat': 0.0,
            'slope': 0.0,
            'rubble': 0.0,
            'stairs': 0.0
        }
        
        # Flat: Low variance, low edge density, uniform color
        if color_variance < 30 and edge_density < 0.15 and value_variance < 40:
            scores['flat'] = 0.8 + (1 - edge_density) * 0.2
        
        # Slope: Directional gradients, moderate variance
        if gradient_std > 0.5 and color_variance > 20:
            slope_score = min(gradient_std / 1.5, 1.0) * 0.6
            scores['slope'] = slope_score + 0.3
        
        # Rubble: High variance, high edge density, irregular patterns
        if color_variance > 35 and edge_density > 0.2:
            rubble_score = min(color_variance / 60, 1.0) * 0.5
            rubble_score += min(edge_density * 2, 0.5)
            scores['rubble'] = rubble_score
        
        # Stairs: Strong horizontal/vertical patterns
        if vertical_edges > 1000 and horizontal_edges > 800:
            pattern_score = min((vertical_edges + horizontal_edges) / 5000, 1.0)
            scores['stairs'] = pattern_score * 0.7 + 0.3
        
        # Ensure at least flat terrain has some score
        if max(scores.values()) < 0.3:
            scores['flat'] = 0.6
        
        # Get prediction
        predicted_class = max(scores, key=scores.get)
        confidence = scores[predicted_class]
        
        return predicted_class, confidence
    
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