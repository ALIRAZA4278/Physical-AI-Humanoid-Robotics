/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros2/architecture',
        'module-1-ros2/first-node',
        'module-1-ros2/urdf',
        'module-1-ros2/environment',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      items: [
        'module-2-simulation/gazebo-setup',
        'module-2-simulation/sdf-modeling',
        'module-2-simulation/sensors',
        'module-2-simulation/unity-viz',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac AI',
      items: [
        'module-3-isaac/isaac-setup',
        'module-3-isaac/perception',
        'module-3-isaac/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Capstone',
      items: [
        'module-4-vla/whisper',
        'module-4-vla/planning',
        'module-4-vla/capstone',
      ],
    },
    'glossary',
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/hardware-requirements',
        'appendix/version-migration',
        'appendix/troubleshooting',
      ],
    },
  ],
};

module.exports = sidebars;
