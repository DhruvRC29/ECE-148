parallel_parking_bot/                  # ROS2 Foxy package root
├── package.xml
├── setup.py
├── README.md                          # (Package documentation)
├── launch/
│   └── parking_demo.launch.py         # Launch file to start all nodes
├── config/
│   └── params.yaml                    # Configuration (e.g. camera and threshold params)
└── src/
    └── parallel_parking_bot/          # Python module for the package
        ├── __init__.py
        ├── vision_node.py             # Node: Vision processing with OpenCV/DepthAI
        ├── parking_controller_node.py # Node: Parking logic & state machine
        ├── vesc_driver_node.py        # Node: VESC motor/servo control interface
        ├── control_utils.py           # Helper library (e.g. predefined maneuvers, constants)
        └── joystick_mapper.py         # (Optional) helper for joystick button mapping
