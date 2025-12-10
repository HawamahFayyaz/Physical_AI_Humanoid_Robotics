import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics book.
 * Organized into 5 modules with 14 total chapters.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 0: Introduction',
      collapsed: false,
      items: [
        'introduction/what-is-physical-ai',
        'introduction/hardware-and-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'ros2/ros2-architecture',
        'ros2/nodes-topics-services',
        'ros2/ros2-packages-launch',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'digital-twin/gazebo-simulation',
        'digital-twin/urdf-robot-description',
        'digital-twin/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac',
      items: [
        'isaac/isaac-sim-introduction',
        'isaac/isaac-ros-perception',
        'isaac/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & Capstone',
      items: [
        'vla-capstone/humanoid-kinematics',
        'vla-capstone/voice-to-action',
        'vla-capstone/capstone-autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
