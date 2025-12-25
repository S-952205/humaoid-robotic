# Physical AI & Humanoid Robotics Course Book with AI Chatbot Integration

[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.2.1-1b1c1d?logo=docusaurus)](https://docusaurus.io/)
[![React](https://img.shields.io/badge/React-18-20232a?logo=react&logoColor=61DAFB)](https://reactjs.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.2-blue?logo=typescript&logoColor=white)](https://www.typescriptlang.org/)
[![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.104-005571?logo=fastapi)](https://fastapi.tiangolo.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A comprehensive educational platform for learning Physical AI and Humanoid Robotics through a simulation-first approach. This course book provides a structured 13-week curriculum covering ROS 2, robot simulation, and Vision-Language-Action (VLA) models, enhanced with an intelligent RAG chatbot for interactive learning.

## 🚀 Features

- **13-Week Structured Curriculum**: Comprehensive learning path from basics to advanced robotics
- **Simulation-First Approach**: Learn with Gazebo, Unity, and NVIDIA Isaac without expensive hardware
- **Hands-On Examples**: Real ROS 2 code examples and practical projects
- **Interactive AI Chatbot**: RAG-powered chatbot that answers questions about course content
- **Chat History**: Persistent chat sessions with Neon Postgres database
- **Selected Text Integration**: Send highlighted text directly to the chatbot
- **Modern Tech Stack**: Built with Docusaurus, React, TypeScript, and Python FastAPI
- **Responsive Design**: Works on all devices with light and dark themes
- **OpenRouter Integration**: Flexible AI model support with OpenRouter

## 🤖 AI Chatbot Features

### RAG (Retrieval Augmented Generation)
- **Context-Aware Responses**: The chatbot understands and responds based on course documentation
- **Document Search**: Searches through course materials to provide accurate answers
- **Semantic Understanding**: Uses vector embeddings to find relevant content

### Chat Interface
- **Floating Widget**: Always accessible chat widget in the bottom-right corner
- **Persistent Sessions**: Chat history saved across browser sessions
- **Session Management**: Unique session IDs stored in localStorage
- **Real-time Interaction**: Instant responses to user queries

### Database Integration
- **Neon Postgres**: Cloud-native Postgres database for chat history
- **Message Persistence**: All conversations saved and retrievable
- **Session Tracking**: Individual chat session management

### Advanced Features
- **Selected Text Functionality**: Highlight text and send directly to chatbot
- **OpenRouter Support**: Configurable AI model selection
- **Error Handling**: Robust error management and user feedback
- **Performance Optimized**: Efficient vector search and response generation

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

### Frontend
- **Framework**: [Docusaurus 3.2.1](https://docusaurus.io/) - Static site generator for documentation
- **Frontend**: [React 18](https://reactjs.org/) - Component-based UI library
- **Language**: [TypeScript](https://www.typescriptlang.org/) - Type-safe JavaScript
- **Styling**: [Tailwind CSS](https://tailwindcss.com/) - Utility-first CSS framework
- **Chat Integration**: [OpenAI ChatKit](https://platform.openai.com/docs/guides/chatkit) - Professional chat widget

### Backend
- **Runtime**: [Python 3.11](https://www.python.org/) - Programming language
- **Framework**: [FastAPI](https://fastapi.tiangolo.com/) - Modern Python web framework
- **Database**: [SQLModel](https://sqlmodel.tiangolo.com/) - SQL database library
- **Vector Database**: [Qdrant](https://qdrant.tech/) - Vector similarity search engine
- **Embeddings**: [OpenAI Embeddings](https://platform.openai.com/docs/guides/embeddings) - Text vectorization
- **Database**: [Neon Postgres](https://neon.tech/) - Serverless Postgres database

### Deployment & Infrastructure
- **Runtime**: [Node.js](https://nodejs.org/) (>=18.0) - JavaScript runtime environment
- **Deployment**: GitHub Pages - Static site hosting
- **AI Provider**: [OpenRouter](https://openrouter.ai/) - AI model routing service

## 🚀 Quick Start

### Prerequisites

- [Node.js](https://nodejs.org/) (version >=18.0)
- [npm](https://www.npmjs.com/) or [yarn](https://yarnpkg.com/)
- [Python](https://www.python.org/) (version >=3.11)
- [uv](https://github.com/astral-sh/uv) or [pip](https://pypi.org/project/pip/) for Python package management

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Syed-Sufyan/humanoid-robot-book.git
   cd humanoid-robot-book
   ```

2. **Install frontend dependencies:**
   ```bash
   npm install
   ```

3. **Set up backend environment:**
   ```bash
   cd backend
   # Create virtual environment (if not already created)
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate

   # Install Python dependencies
   uv pip install -r requirements.txt
   # Or if you don't have uv: pip install -r requirements.txt
   ```

4. **Configure environment variables:**
   Create a `.env` file in the `backend/` directory with the following:
   ```env
   OPENAI_API_KEY=your_openai_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_database_url
   OPENROUTER_API_KEY=your_openrouter_api_key
   ```

5. **Start the backend server:**
   ```bash
   cd backend
   python main.py
   # Or use: uv run main.py
   ```

6. **In a new terminal, start the frontend development server:**
   ```bash
   npm start
   ```

7. **Open your browser:**
   Visit [http://localhost:3000/humaoid-robotic/](http://localhost:3000/humaoid-robotic/) to view the course book with the integrated chatbot

### Build for Production

**Frontend:**
```bash
npm run build
```

**Backend:**
```bash
cd backend
# Make sure environment variables are set
python main.py
```

The built site will be in the `build/` directory and can be deployed to any static hosting service.

## 📁 Project Structure

```
humanoid-robot-book/
├── backend/                   # Python backend with AI integration
│   ├── api/                  # FastAPI endpoints
│   │   └── chat.py          # Chat and RAG endpoints
│   ├── models/               # SQLModel database models
│   ├── services/             # Business logic services
│   │   ├── chat_history_service.py  # Chat history management
│   │   ├── rag_service.py   # RAG functionality
│   │   └── chat_service.py  # Chat business logic
│   ├── agent.py             # AI agent with RAG capabilities
│   ├── main.py              # FastAPI application entry point
│   ├── config.py            # Configuration settings
│   ├── crawler.py           # Content crawling utilities
│   ├── embedder.py          # Vector embedding functions
│   ├── vector_store.py      # Qdrant vector database operations
│   ├── database.py          # Database connection utilities
│   ├── requirements.txt     # Python dependencies
│   └── .env                 # Environment variables
├── blog/                    # Blog posts and updates
├── docs/                    # Course documentation
│   ├── chatbot_improvements.md    # Chatbot enhancement documentation
│   ├── chatkit_neon_integration.md # ChatKit and Neon integration guide
│   ├── openrouter_configuration.md # OpenRouter setup documentation
│   ├── 01-intro/           # Module 1: Physical AI & Humanoid Basics
│   ├── 02-ros2/            # Module 2: ROS 2 Fundamentals
│   ├── 03-simulation/      # Module 3: Robot Simulation
│   ├── 04-isaac/           # Module 4: NVIDIA Isaac Simulation
│   ├── 05-vla/             # Module 5: Vision-Language-Action Models
│   ├── capstone/           # Capstone project materials
│   ├── hardware-guide/     # Hardware recommendations
│   └── intro.md            # Main course introduction
├── src/                     # Source files
│   ├── components/          # React components
│   │   └── ChatWidget/     # AI chat widget component
│   ├── css/                # Global styles
│   ├── pages/              # Page components
│   └── theme/              # Docusaurus theme customization
├── static/                  # Static assets
├── .github/                 # GitHub workflows
│   └── workflows/
│       └── deploy.yml       # GitHub Pages deployment
├── docusaurus.config.js     # Docusaurus configuration
├── sidebar.js               # Navigation sidebar
├── package.json            # Project dependencies and scripts
└── README.md              # This file
```

## 🤖 Chatbot Integration Guide

### How the RAG Chatbot Works

1. **Query Processing**: When a user asks a question, the query is sent to the backend
2. **Vector Search**: The system performs semantic search in the Qdrant vector database
3. **Context Retrieval**: Relevant course content is retrieved based on the query
4. **AI Response**: An AI model generates a response using the retrieved context
5. **Response Delivery**: The answer is returned to the frontend with sources

### Using the Chat Features

- **Start a conversation**: Click the chat bubble in the bottom-right corner
- **Send selected text**: Highlight text on any page and click "Selected Text" button
- **Continue sessions**: Your chat history persists across browser sessions
- **Ask course questions**: Inquire about any topic covered in the course materials

### Database Schema

The chat history is stored in Neon Postgres with the following structure:
- **sessions**: Individual chat sessions with unique IDs
- **messages**: Individual messages with roles (user/assistant), content, and timestamps

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

### AI-Powered Learning
Learn how to use the integrated RAG chatbot for enhanced learning experiences.

## 🔧 API Endpoints

The backend provides the following API endpoints:

- `POST /chat/query` - Process a query with RAG capabilities
- `GET /chat/history/{session_id}` - Retrieve chat history for a session
- `POST /chat/history` - Save a message to chat history

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
- Maintain the RAG chatbot functionality when making changes

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support & Contact

- **GitHub Repository**: [https://github.com/Syed-Sufyan/humanoid-robot-book](https://github.com/Syed-Sufyan/humanoid-robot-book)
- **Issues**: Report bugs or request features using the GitHub Issues tab
- **Documentation**: Check the course documentation for detailed information

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/) - an excellent documentation framework
- Powered by [React](https://reactjs.org/) and [TypeScript](https://www.typescriptlang.org/)
- Enhanced with [FastAPI](https://fastapi.tiangolo.com/) for backend services
- Vector search powered by [Qdrant](https://qdrant.tech/)
- Database hosting by [Neon](https://neon.tech/)
- AI models accessible through [OpenRouter](https://openrouter.ai/)

---
**Ready to start your journey into Physical AI and Humanoid Robotics?** [Begin the course](http://localhost:3000/humaoid-robotic/intro) today and interact with our AI-powered learning assistant!