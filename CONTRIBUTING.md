# Contributing to Terrain-Aware Locomotion

Thank you for your interest in contributing to this project! This document provides guidelines for contributing.

## Code of Conduct

Please read and follow our [Code of Conduct](CODE_OF_CONDUCT.md).

## How to Contribute

### Reporting Bugs

If you find a bug, please create an issue with:
- A clear, descriptive title
- Steps to reproduce the issue
- Expected behavior
- Actual behavior
- Your environment (OS, ROS2 version, Python version)
- Screenshots if applicable

### Suggesting Features

Feature requests are welcome! Please create an issue with:
- A clear, descriptive title
- Detailed description of the proposed feature
- Use cases and examples
- Any alternative solutions you've considered

### Pull Requests

1. **Fork the repository** and create your branch from `main`
2. **Write clear commit messages** following conventional commits format:
   - `feat: add new terrain classifier`
   - `fix: correct IK solution for rear legs`
   - `docs: update installation guide`
   - `test: add tests for gait generator`

3. **Follow the code style**:
   - Use Python 3.8+ features
   - Follow PEP 8 style guide
   - Use Black for code formatting (`black .`)
   - Add type hints where possible
   - Include docstrings for all public methods

4. **Write tests**:
   - Add unit tests for new functionality
   - Ensure all tests pass: `pytest`
   - Maintain or improve code coverage

5. **Update documentation**:
   - Update README.md if needed
   - Add docstrings to new functions/classes
   - Update API documentation if applicable

6. **Run linters**:
   ```bash
   black src/
   flake8 src/
   ```

7. **Submit the PR**:
   - Reference any related issues
   - Describe your changes clearly
   - Include screenshots for UI changes
   - Wait for review and address feedback

## Development Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/ansh1113/terrain-aware-locomotion.git
   cd terrain-aware-locomotion
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Build the ROS2 workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Run tests:
   ```bash
   colcon test
   colcon test-result --verbose
   ```

## Project Structure

```
terrain-aware-locomotion/
├── src/
│   └── terrain_locomotion/
│       ├── terrain_locomotion/        # Python package
│       │   ├── perception/           # Perception modules
│       │   ├── planning/             # Planning modules
│       │   └── control/              # Control modules
│       ├── launch/                   # Launch files
│       ├── config/                   # Configuration files
│       ├── worlds/                   # Gazebo worlds
│       └── test/                     # Tests
├── docs/                             # Documentation
├── .github/                          # GitHub workflows
└── README.md
```

## Testing

- Write unit tests for all new functionality
- Use pytest for testing: `pytest src/terrain_locomotion/test/`
- Aim for >80% code coverage
- Test on both simulated and real hardware when possible

## Code Review Process

1. Maintainers will review your PR
2. Address review comments
3. Once approved, a maintainer will merge your PR

## Questions?

Feel free to open an issue for questions or contact the maintainers directly.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

Thank you for contributing! 🚀
