#!/bin/bash

echo "Deploying project shortcut to desktop from: " `pwd`
cp -f config/niro_demo_node_manager.desktop ~/Desktop
chmod +x ~/Desktop/niro_demo_node_manager.desktop
sed -i -- 's+WORKSPACE+'$1'+g' ~/Desktop/niro_demo_node_manager.desktop
sed -i -- 's+PACKAGE+'$2'+g' ~/Desktop/niro_demo_node_manager.desktop

cp -f config/niro_joy_node_manager.desktop ~/Desktop
chmod +x ~/Desktop/niro_joy_node_manager.desktop
sed -i -- 's+WORKSPACE+'$1'+g' ~/Desktop/niro_joy_node_manager.desktop
sed -i -- 's+PACKAGE+'$2'+g' ~/Desktop/niro_joy_node_manager.desktop

cp -f config/topic_diagnostic.desktop ~/Desktop
chmod +x ~/Desktop/topic_diagnostic.desktop
sed -i -- 's+WORKSPACE+'$1'+g' ~/Desktop/topic_diagnostic.desktop
sed -i -- 's+PACKAGE+'$2'+g' ~/Desktop/topic_diagnostic.desktop
