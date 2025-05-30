entry_points={
    'console_scripts': [
        'vision_node = parallel_parking_bot.vision_node:main',
        'parking_controller_node = parallel_parking_bot.parking_controller_node:main',
        'vesc_driver_node = parallel_parking_bot.vesc_driver_node:main',
    ],
},
