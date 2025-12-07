/**
 * Creating a sidebar enables you to:
 * - Create an ordered group of docs
 * - Render a sidebar for each doc of that group
 * - HMR (Hot Module Replacement) when editing docs
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Physical AI & Humanoid Basics',
      items: [
        'intro/week-01-chapter-1',
        'intro/week-02-chapter-2',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Fundamentals',
      items: [
        'ros2/week-03-chapter-3',
        'ros2/week-04-chapter-4',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Robot Simulation',
      items: [
        'simulation/week-05-chapter-5',
        'simulation/week-06-chapter-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: NVIDIA Isaac Simulation',
      items: [
        'isaac/week-07-chapter-7',
        'isaac/week-08-chapter-8',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Vision-Language-Action Models',
      items: [
        'vla/week-09-chapter-9',
        'vla/week-10-chapter-10',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Guide',
      items: [
        'hardware-guide/recommended-hardware',
      ],
    },
  ],
};

export default sidebars;