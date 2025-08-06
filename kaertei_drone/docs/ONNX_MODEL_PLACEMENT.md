# ONNX Model Placement Guide - KAERTEI 2025 FAIO

## 📁 Model Directory Structure
```
kaertei_drone/
├── models/           📂 PLACE ONNX MODELS HERE
│   ├── yolo/
│   │   ├── yolov8_objects.onnx      # Object detection
│   │   ├── yolov8_dropzone.onnx     # Dropzone detection  
│   │   └── yolov8_exit.onnx         # Exit gate detection
│   └── custom/
│       └── kaertei_items.onnx       # Competition-specific items
└── vision/
    └── unified_vision_system.py     # Uses models from models/ folder
```

## 🎯 Model Types & Usage

### 1. YOLO Object Detection Models
- **Location**: `models/yolo/`
- **Format**: `.onnx` files
- **Usage**: Object detection for items, dropzones, exit gates
- **Loaded by**: `unified_vision_system.py`

### 2. Custom Training Models  
- **Location**: `models/custom/`
- **Format**: `.onnx` files
- **Usage**: Competition-specific object recognition
- **Loaded by**: Custom detection nodes

## 🔧 Configuration

### Update Model Paths in Code:
```python
# In unified_vision_system.py
YOLO_MODEL_PATH = "models/yolo/yolov8_objects.onnx"
CUSTOM_MODEL_PATH = "models/custom/kaertei_items.onnx"
```

### Hardware Requirements:
- **CPU**: Intel i5 or equivalent
- **RAM**: 4GB minimum for ONNX inference
- **Storage**: 100MB for model files

## 🚀 Quick Setup

1. **Create models directory:**
   ```bash
   mkdir -p models/yolo models/custom
   ```

2. **Place your ONNX models:**
   ```bash
   cp your_yolo_model.onnx models/yolo/
   cp custom_model.onnx models/custom/
   ```

3. **Update configuration:**
   - Edit `unified_vision_system.py`
   - Set correct model paths
   - Adjust confidence thresholds

4. **Test the system:**
   ```bash
   ./run_simplified_mission.sh debug
   ```

## ✅ Current System Status

After cleanup, the vision system now uses:
- ✅ `unified_vision_system.py` - Single unified vision node
- ✅ `simplified_mission_control.py` - Main mission controller
- ❌ Old checkpoint_mission_*.py - REMOVED
- ❌ Old individual vision detectors - REMOVED

## 📝 Notes

- Models must be in ONNX format for cross-platform compatibility
- Place models in `models/` directory, not in package root
- Use relative paths in configuration files
- Test thoroughly after model placement

---
*Updated after codebase cleanup - January 2025*
