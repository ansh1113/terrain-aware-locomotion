"""CNN architectures for terrain classification.

This module provides lightweight CNN models suitable for real-time terrain
classification on quadruped robots.
"""

import torch
import torch.nn as nn
import torchvision.models as models
from typing import Dict, Any


class TerrainCNN(nn.Module):
    """Lightweight custom CNN for terrain classification.

    This model is designed to be fast and efficient for real-time inference
    on embedded systems while maintaining good classification accuracy.

    Args:
        num_classes: Number of terrain classes (default: 4)
        input_channels: Number of input image channels (default: 3 for RGB)
        dropout_rate: Dropout rate for regularization (default: 0.5)
    """

    def __init__(
        self,
        num_classes: int = 4,
        input_channels: int = 3,
        dropout_rate: float = 0.5
    ):
        super(TerrainCNN, self).__init__()

        self.num_classes = num_classes

        # Feature extraction layers
        self.features = nn.Sequential(
            # Block 1: 224x224x3 -> 112x112x32
            nn.Conv2d(input_channels, 32, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),

            # Block 2: 112x112x32 -> 56x56x64
            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),

            # Block 3: 56x56x64 -> 28x28x128
            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),

            # Block 4: 28x28x128 -> 14x14x256
            nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),

            # Block 5: 14x14x256 -> 7x7x256
            nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),

            # Global average pooling: 7x7x256 -> 1x1x256
            nn.AdaptiveAvgPool2d((1, 1))
        )

        # Classification head
        self.classifier = nn.Sequential(
            nn.Dropout(dropout_rate),
            nn.Linear(256, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout_rate / 2),
            nn.Linear(128, num_classes)
        )

        # Initialize weights
        self._initialize_weights()

    def _initialize_weights(self):
        """Initialize model weights using He initialization."""
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(
                    m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass through the network.

        Args:
            x: Input tensor of shape (batch_size, channels, height, width)

        Returns:
            Output logits of shape (batch_size, num_classes)
        """
        x = self.features(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x

    def get_features(self, x: torch.Tensor) -> torch.Tensor:
        """Extract feature vector before classification.

        Args:
            x: Input tensor of shape (batch_size, channels, height, width)

        Returns:
            Feature vector of shape (batch_size, 256)
        """
        x = self.features(x)
        x = torch.flatten(x, 1)
        return x


class MobileNetV2Classifier(nn.Module):
    """MobileNetV2-based terrain classifier.

    Uses pretrained MobileNetV2 as backbone for improved accuracy.
    Suitable for systems with more computational resources.

    Args:
        num_classes: Number of terrain classes (default: 4)
        pretrained: Whether to use ImageNet pretrained weights (default: True)
        freeze_backbone: Whether to freeze backbone weights (default: False)
    """

    def __init__(
        self,
        num_classes: int = 4,
        pretrained: bool = True,
        freeze_backbone: bool = False
    ):
        super(MobileNetV2Classifier, self).__init__()

        self.num_classes = num_classes

        # Load MobileNetV2 backbone
        mobilenet = models.mobilenet_v2(pretrained=pretrained)

        # Extract feature extractor (remove classifier)
        self.features = mobilenet.features

        # Optionally freeze backbone
        if freeze_backbone:
            for param in self.features.parameters():
                param.requires_grad = False

        # Replace classifier
        # MobileNetV2 outputs 1280 features
        self.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(1280, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(512, num_classes)
        )

        # Initialize new classifier layers
        self._initialize_classifier()

    def _initialize_classifier(self):
        """Initialize classifier weights."""
        for m in self.classifier.modules():
            if isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass through the network.

        Args:
            x: Input tensor of shape (batch_size, 3, 224, 224)

        Returns:
            Output logits of shape (batch_size, num_classes)
        """
        x = self.features(x)
        # Global average pooling
        x = nn.functional.adaptive_avg_pool2d(x, (1, 1))
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x

    def unfreeze_backbone(self):
        """Unfreeze backbone for fine-tuning."""
        for param in self.features.parameters():
            param.requires_grad = True


def load_model(
    model_path: str,
    model_type: str = 'custom',
    num_classes: int = 4,
    device: str = 'cpu'
) -> nn.Module:
    """Load a trained terrain classification model.

    Args:
        model_path: Path to the saved model weights (.pth file)
        model_type: Type of model ('custom' or 'mobilenet')
        num_classes: Number of terrain classes
        device: Device to load model on ('cpu' or 'cuda')

    Returns:
        Loaded model in evaluation mode

    Raises:
        ValueError: If model_type is not recognized
        FileNotFoundError: If model_path does not exist
    """
    # Create model
    if model_type == 'custom':
        model = TerrainCNN(num_classes=num_classes)
    elif model_type == 'mobilenet':
        model = MobileNetV2Classifier(
            num_classes=num_classes, pretrained=False)
    else:
        raise ValueError(f"Unknown model type: {model_type}")

    # Load weights
    try:
        state_dict = torch.load(model_path, map_location=device)
        model.load_state_dict(state_dict)
    except FileNotFoundError:
        raise FileNotFoundError(f"Model file not found: {model_path}")
    except Exception as e:
        raise RuntimeError(f"Error loading model: {str(e)}")

    # Set to evaluation mode
    model.eval()
    model.to(device)

    return model


def get_model_info(model: nn.Module) -> Dict[str, Any]:
    """Get information about a model.

    Args:
        model: PyTorch model

    Returns:
        Dictionary with model information (num_params, model_type, etc.)
    """
    num_params = sum(p.numel() for p in model.parameters())
    num_trainable = sum(p.numel()
                        for p in model.parameters() if p.requires_grad)

    model_type = 'unknown'
    if isinstance(model, TerrainCNN):
        model_type = 'custom_cnn'
    elif isinstance(model, MobileNetV2Classifier):
        model_type = 'mobilenet_v2'

    return {
        'model_type': model_type,
        'total_parameters': num_params,
        'trainable_parameters': num_trainable,
        'num_classes': getattr(model, 'num_classes', None)
    }


if __name__ == '__main__':
    # Quick test of model architectures
    print("Testing TerrainCNN...")
    custom_model = TerrainCNN(num_classes=4)
    test_input = torch.randn(1, 3, 224, 224)
    output = custom_model(test_input)
    print(f"Output shape: {output.shape}")
    print(f"Model info: {get_model_info(custom_model)}")

    print("\nTesting MobileNetV2Classifier...")
    mobilenet_model = MobileNetV2Classifier(num_classes=4, pretrained=False)
    output = mobilenet_model(test_input)
    print(f"Output shape: {output.shape}")
    print(f"Model info: {get_model_info(mobilenet_model)}")
