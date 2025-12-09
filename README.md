# Physical AI & Humanoid Robotics Course Book

[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.2.1-1b1c1d?logo=docusaurus)](https://docusaurus.io/)
[![React](https://img.shields.io/badge/React-18-20232a?logo=react&logoColor=61DAFB)](https://reactjs.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.2-blue?logo=typescript&logoColor=white)](https://www.typescriptlang.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A comprehensive educational platform for learning Physical AI and Humanoid Robotics through a simulation-first approach. This course book provides a structured 13-week curriculum covering ROS 2, robot simulation, and Vision-Language-Action (VLA) models.

## 🚀 Features

- **13-Week Structured Curriculum**: Comprehensive learning path from basics to advanced robotics
- **Simulation-First Approach**: Learn with Gazebo, Unity, and NVIDIA Isaac without expensive hardware
- **Hands-On Examples**: Real ROS 2 code examples and practical projects
- **Modern Tech Stack**: Built with Docusaurus, React, and TypeScript
- **Responsive Design**: Works on all devices with light and dark themes

## 📚 Course Modules

| Module | Topic | Duration | Focus |
|--------|-------|----------|-------|
| 1 | Physical AI & Humanoid Basics | Weeks 1-2 | Foundations of physical AI and humanoid robot design |
| 2 | ROS 2 Fundamentals | Weeks 3-4 | Master Robot Operating System 2 as robot's "nervous system" |
| 3 | Robot Simulation | Weeks 5-6 | Explore simulation platforms (Gazebo, Unity, NVIDIA Isaac) |
| 4 | NVIDIA Isaac Simulation | Weeks 7-8 | Deep dive into NVIDIA's advanced robotics simulation |
| 5 | Vision-Language-Action Models | Weeks 9-10 | Implement cutting-edge AI for robot control |
| 6 | Capstone Project | Weeks 11-13 | Integrate everything into a complete robot system |
| 7 | Hardware Guide | Reference | Recommended components for real-world deployment |

## 🛠️ Tech Stack

- **Framework**: [Docusaurus 3.2.1](https://docusaurus.io/) - Static site generator for documentation
- **Frontend**: [React 18](https://reactjs.org/) - Component-based UI library
- **Language**: [TypeScript](https://www.typescriptlang.org/) - Type-safe JavaScript
- **Runtime**: [Node.js](https://nodejs.org/) (>=18.0) - JavaScript runtime environment
- **Styling**: CSS Modules with [Infima](https://infima.dev/) - Component-scoped styling
- **Code Highlighting**: [Prism React Renderer](https://github.com/FormidableLabs/prism-react-renderer) - Syntax highlighting
- **Deployment**: GitHub Pages - Static site hosting

## 🚀 Quick Start

### Prerequisites

- [Node.js](https://nodejs.org/) (version >=18.0)
- [npm](https://www.npmjs.com/) or [yarn](https://yarnpkg.com/)

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Syed-Sufyan/humanoid-robot-book.git
   cd humanoid-robot-book
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm start
   ```

4. **Open your browser:**
   Visit [http://localhost:3000](http://localhost:3000) to view the course book

### Build for Production

```bash
npm run build
```

The built site will be in the `build/` directory and can be deployed to any static hosting service.

## 📁 Project Structure

```
humanoid-robot-book/
├── blog/                    # Blog posts and updates
├── docs/                    # Course documentation
│   ├── 01-intro/           # Module 1: Physical AI & Humanoid Basics
│   ├── 02-ros2/            # Module 2: ROS 2 Fundamentals
│   ├── 03-simulation/      # Module 3: Robot Simulation
│   ├── 04-isaac/           # Module 4: NVIDIA Isaac Simulation
│   ├── 05-vla/             # Module 5: Vision-Language-Action Models
│   ├── capstone/           # Capstone project materials
│   ├── hardware-guide/     # Hardware recommendations
│   └── intro.md            # Main course introduction
├── src/                     # Source files
│   ├── css/                # Global styles
│   └── pages/              # Page components
├── static/                  # Static assets
├── .github/                 # GitHub workflows
│   └── workflows/
│       └── deploy.yml       # GitHub Pages deployment
├── docusaurus.config.js     # Docusaurus configuration
├── sidebar.js               # Navigation sidebar
├── package.json            # Project dependencies and scripts
└── README.md              # This file
```

## 🎓 What You'll Learn

### Module 1: Physical AI & Humanoid Basics
Understanding the foundations of physical AI, humanoid robot design, and key concepts.

### Module 2: ROS 2 Fundamentals
Master ROS 2 as the central communication framework for robot systems.

### Module 3: Robot Simulation
Explore different simulation platforms (Gazebo, Unity, NVIDIA Isaac).

### Module 4: NVIDIA Isaac Simulation
Deep dive into NVIDIA's advanced robotics simulation platform.

### Module 5: Vision-Language-Action Models
Implement state-of-the-art AI models for robot control via natural language.

### Capstone Project
Integrate everything you've learned into a complete, working humanoid robot system.

### Hardware Guide
Understand the recommended hardware components for real-world deployment.

## 🤝 Contributing

We welcome contributions to improve the course content and platform! Here's how you can help:

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/amazing-feature`)
3. **Make your changes**
4. **Commit your changes** (`git commit -m 'Add some amazing feature'`)
5. **Push to the branch** (`git push origin feature/amazing-feature`)
6. **Open a Pull Request**

### Development Guidelines

- Write clear, concise documentation
- Follow the existing code style
- Test your changes thoroughly
- Update the sidebar if adding new content
- Follow the 13-week curriculum structure

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support & Contact

- **GitHub Repository**: [https://github.com/Syed-Sufyan/humanoid-robot-book](https://github.com/Syed-Sufyan/humanoid-robot-book)
- **Issues**: Report bugs or request features using the GitHub Issues tab
- **Documentation**: Check the course documentation for detailed information

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/) - an excellent documentation framework
- Powered by [React](https://reactjs.org/) and [TypeScript](https://www.typescriptlang.org/)

---

**Ready to start your journey into Physical AI and Humanoid Robotics?** [Begin the course](http://localhost:3000/intro) today!