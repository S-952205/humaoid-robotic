import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn Physical AI and Humanoid Robotics with ROS 2, Simulation, and VLA Models"
    >
      <main>
        <section className={styles.hero}>
          <div className={styles.heroContent}>
            <h1 className={styles.title}>Physical AI & Humanoid Robotics</h1>
            <p className={styles.subtitle}>
              Learn from scratch with simulation-first approach
            </p>
            <p className={styles.description}>
              Master ROS 2, robot simulation, and Vision-Language-Action models through hands-on learning
            </p>
            <div className={styles.buttons}>
              <Link
                className={styles.buttonPrimary}
                to="/intro"
              >
                Start Learning →
              </Link>
              <Link
                className={styles.buttonSecondary}
                to="/blog"
              >
                Read Blog
              </Link>
            </div>
          </div>

          <div className={styles.heroImage}>
            <svg viewBox="0 0 200 200" xmlns="http://www.w3.org/2000/svg">
              {/* Robot illustration */}
              <circle cx="100" cy="70" r="35" fill="#2563eb" stroke="#1e40af" strokeWidth="2"/>
              <circle cx="85" cy="65" r="6" fill="#fff"/>
              <circle cx="115" cy="65" r="6" fill="#fff"/>
              <circle cx="85" cy="65" r="3" fill="#000"/>
              <circle cx="115" cy="65" r="3" fill="#000"/>
              <rect x="75" y="105" width="50" height="60" fill="#2563eb" stroke="#1e40af" strokeWidth="2" rx="5"/>
              <rect x="40" y="120" width="35" height="15" fill="#2563eb" stroke="#1e40af" strokeWidth="2" rx="7"/>
              <rect x="125" y="120" width="35" height="15" fill="#2563eb" stroke="#1e40af" strokeWidth="2" rx="7"/>
              <rect x="80" y="165" width="12" height="25" fill="#2563eb" stroke="#1e40af" strokeWidth="2" rx="6"/>
              <rect x="108" y="165" width="12" height="25" fill="#2563eb" stroke="#1e40af" strokeWidth="2" rx="6"/>
              <circle cx="100" cy="130" r="8" fill="#dbeafe"/>
            </svg>
          </div>
        </section>

        <section className={styles.features}>
          <div className={styles.feature}>
            <div className={styles.featureIcon}>🤖</div>
            <h3>Simulation-First</h3>
            <p>Learn with Gazebo, Unity, and NVIDIA Isaac without expensive hardware</p>
          </div>

          <div className={styles.feature}>
            <div className={styles.featureIcon}>📚</div>
            <h3>Complete Course</h3>
            <p>13 weeks of structured learning from basics to advanced robotics</p>
          </div>

          <div className={styles.feature}>
            <div className={styles.featureIcon}>💻</div>
            <h3>Hands-On Code</h3>
            <p>Real ROS 2 examples and practical projects you can run immediately</p>
          </div>

          <div className={styles.feature}>
            <div className={styles.featureIcon}>🎓</div>
            <h3>Expert Content</h3>
            <p>Learn from verified, accurate technical content and best practices</p>
          </div>
        </section>

        <section className={styles.modules}>
          <h2>Course Modules</h2>
          <div className={styles.moduleGrid}>
            <div className={styles.moduleCard}>
              <h4>Module 1</h4>
              <p>Physical AI & Humanoid Basics</p>
              <p className={styles.duration}>Weeks 1-2</p>
            </div>
            <div className={styles.moduleCard}>
              <h4>Module 2</h4>
              <p>ROS 2 Fundamentals</p>
              <p className={styles.duration}>Weeks 3-4</p>
            </div>
            <div className={styles.moduleCard}>
              <h4>Module 3</h4>
              <p>Robot Simulation</p>
              <p className={styles.duration}>Weeks 5-6</p>
            </div>
            <div className={styles.moduleCard}>
              <h4>Module 4</h4>
              <p>NVIDIA Isaac Simulation</p>
              <p className={styles.duration}>Weeks 7-8</p>
            </div>
            <div className={styles.moduleCard}>
              <h4>Module 5</h4>
              <p>Vision-Language-Action Models</p>
              <p className={styles.duration}>Weeks 9-10</p>
            </div>
            <div className={styles.moduleCard}>
              <h4>Capstone</h4>
              <p>Complete Project</p>
              <p className={styles.duration}>Weeks 11-13</p>
            </div>
          </div>
        </section>

        <section className={styles.cta}>
          <h2>Ready to Start Learning?</h2>
          <p>Join thousands of learners mastering Physical AI and Humanoid Robotics</p>
          <Link className={styles.ctaButton} to="/intro">
            Begin the Course
          </Link>
        </section>
      </main>
    </Layout>
  );
}
