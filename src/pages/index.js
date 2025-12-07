import React, { useContext } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useHistory, useLocation } from '@docusaurus/router';
import Layout from '@theme/Layout';
import { AuthContext } from '../contexts/AuthContext';
import styles from './index.module.css';

function HeroSection() {
  const introUrl = useBaseUrl('/docs/intro');
  const signupUrl = useBaseUrl('/signup');
  const { isAuthenticated } = useContext(AuthContext);
  const location = useLocation();
  const history = useHistory();
  
  const handleGetStarted = () => {
    if (isAuthenticated) {
      history.push(introUrl);
    } else {
      // Add redirect parameter to return to home after signup
      history.push(`${signupUrl}?redirect=${encodeURIComponent(location.pathname)}`);
    }
  };
  
  return (
    <div className={styles.hero}>
      <div className={styles.heroContent}>
        <div className={styles.badge}>
          <span className={styles.badgeIcon}>🎓</span>
          PANAVERSITY PHYSICAL AI BOOK SERIES
        </div>
        
        <h1 className={styles.heroTitle}>
          Master Physical AI &<br />Humanoid Robotics
        </h1>
        
        <p className={styles.heroSubtitle}>
          Build autonomous robots from scratch. Learn ROS 2, NVIDIA Isaac Sim, and AI-native development in a complete 13-week hands-on curriculum.
        </p>
        
        <div className={styles.heroStats}>
          <div className={styles.statItem}>
            <div className={styles.statNumber}>7</div>
            <div className={styles.statLabel}>Parts</div>
          </div>
          <div className={styles.statDivider}></div>
          <div className={styles.statItem}>
            <div className={styles.statNumber}>13</div>
            <div className={styles.statLabel}>Weeks</div>
          </div>
          <div className={styles.statDivider}></div>
          <div className={styles.statItem}>
            <div className={styles.statNumber}>100%</div>
            <div className={styles.statLabel}>Free</div>
          </div>
        </div>

        <div className={styles.ctaButtons}>
          <button 
            className={styles.ctaPrimary} 
            onClick={handleGetStarted}
          >
            {isAuthenticated ? '📖 Continue Learning' : '🚀 Get Started Free'}
          </button>
          <a
            className={styles.ctaSecondary}
            href="https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook"
            target="_blank"
            rel="noopener noreferrer">
            ⭐ View on GitHub
          </a>
        </div>
        
        <div className={styles.featureBadges}>
          <div className={styles.featureBadge}>
            <span>✨</span> Open Source
          </div>
          <div className={styles.featureBadge}>
            <span>🤖</span> AI Co-Learning
          </div>
          <div className={styles.featureBadge}>
            <span>🏗️</span> Industry Tools
          </div>
          <div className={styles.featureBadge}>
            <span>💰</span> Budget Friendly
          </div>
        </div>
      </div>
      
      {/* Animated Background Elements */}
      <div className={styles.heroBackground}>
        <div className={styles.heroCircle1}></div>
        <div className={styles.heroCircle2}></div>
        <div className={styles.heroCircle3}></div>
      </div>
    </div>
  );
}

function SpectrumCard({ icon, level, title, description, items, role, impact, badge, borderColor }) {
  return (
    <div className={styles.spectrumCard} style={borderColor ? { border: `2px solid ${borderColor}` } : {}}>
      {badge && <div className={styles.cardBadge} style={{ background: badge.color }}>{badge.text}</div>}
      <div className={styles.cardIcon}>{icon}</div>
      <h3>{level}: {title}</h3>
      <p><strong>What it is:</strong> {description}</p>
      <ul className={styles.cardList}>
        {items.map((item, idx) => (
          <li key={idx}>✓ {item}</li>
        ))}
      </ul>
      <p className={styles.cardRole}><strong>Your role:</strong> {role}</p>
      <p className={styles.cardImpact} style={borderColor ? { color: borderColor } : {}}><strong>Impact:</strong> {impact}</p>
    </div>
  );
}

function FeatureCard({ icon, title, description, badge }) {
  return (
    <div className={styles.featureCard}>
      {badge && <div className={styles.cardBadge}>{badge}</div>}
      <div className={styles.featureIcon}>{icon}</div>
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}

function CoursePartCard({ number, title, description, weeks, linkUrl }) {
  return (
    <Link to={linkUrl} className={styles.coursePartCard}>
      <div className={styles.partNumber}>Part {number}</div>
      <h3 className={styles.partTitle}>{title}</h3>
      <p className={styles.partDescription}>{description}</p>
      <div className={styles.partMeta}>
        <span className={styles.partWeeks}>📅 {weeks}</span>
        <span className={styles.partLink}>Explore →</span>
      </div>
    </Link>
  );
}

function MaturityLevel({ level, title, approach, description, focus, productivity, isTarget }) {
  return (
    <div className={isTarget ? styles.maturityLevelTarget : styles.maturityLevel}>
      {isTarget && <div className={styles.targetBadge}>TEXTBOOK TARGET</div>}
      <div className={styles.levelNumber}>{level}</div>
      <div className={styles.levelContent}>
        <h3>{title}</h3>
        <p className={styles.levelApproach}><strong>Approach:</strong> {approach}</p>
        <p>{description}</p>
        <div className={styles.levelMeta}>
          <span className={styles.focusBadge}>{focus}</span>
          <span className={styles.productivity}>{productivity}</span>
        </div>
      </div>
    </div>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  const introUrl = useBaseUrl('/docs/intro');
  const assessmentsUrl = useBaseUrl('/docs/assessments');
  const hardwareUrl = useBaseUrl('/docs/appendix/hardware-recommendations');
  const setupUrl = useBaseUrl('/docs/appendix/setup-instructions');
  const troubleshootingUrl = useBaseUrl('/docs/appendix/troubleshooting');
  
  return (
    <Layout
      title="Home"
      description="Master embodied intelligence with ROS 2, humanoid robotics, and AI-native development">
      <HeroSection />
      
      <main className={styles.main}>
        {/* Spectrum Section */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>The Physical AI Development Spectrum</h2>
          <p className={styles.sectionSubtitle}>
            Three distinct levels of robotics intelligence. This textbook focuses on Level 2 & 3, where AI becomes a co-creator and core system component.
          </p>
          
          <div className={styles.spectrumGrid}>
            <SpectrumCard
              icon="🛠️"
              level="Level 1"
              title="Teleoperation & Scripts"
              description="Manual control or hardcoded behaviors"
              items={[
                'Joystick teleoperation',
                'Pre-programmed sequences',
                'If-then logic'
              ]}
              role="You control everything manually"
              impact="Baseline capabilities"
            />
            
            <SpectrumCard
              icon="🤖"
              level="Level 2"
              title="AI-Assisted Robotics"
              description="AI as your development and runtime partner"
              items={[
                'Spec-driven robot design',
                'AI generates ROS 2 code',
                'Computer vision for perception',
                'Path planning algorithms'
              ]}
              role="Architect systems; AI implements; you validate"
              impact="5-10x faster development"
              badge={{ text: 'PRIMARY FOCUS', color: '#667eea' }}
              borderColor="#667eea"
            />
            
            <SpectrumCard
              icon="🧠"
              level="Level 3"
              title="Embodied Intelligence"
              description="AI reasoning as the robot's core capability"
              items={[
                'Natural language commands',
                'Autonomous task planning',
                'Learning from interaction',
                'Multi-agent coordination'
              ]}
              role="Design agent architectures and guardrails"
              impact="New capabilities impossible before"
              badge={{ text: 'ADVANCED', color: '#764ba2' }}
            />
          </div>
        </section>

        {/* Features Section */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>What Makes This Textbook Different</h2>
          
          <div className={styles.featuresGrid}>
            <FeatureCard
              icon="🎓"
              title="Hands-On Learning"
              description="Every chapter includes runnable code examples, practical exercises, and real robot simulations. Learn by building, not just reading."
            />
            
            <FeatureCard
              icon="🏗️"
              title="Complete 13-Week System"
              description="From fundamentals to deployment. Structured curriculum takes you from zero to building autonomous humanoid robots step-by-step."
            />
            
            <FeatureCard
              icon="🔧"
              title="Industry-Standard Tools"
              description="Master ROS 2, Gazebo, NVIDIA Isaac Sim, MoveIt2, and Nav2 — the same stack used at Boston Dynamics, Tesla Bot, and Agility Robotics."
            />
            
            <FeatureCard
              icon="🤝"
              title="AI Co-Learning Approach"
              description="Learn to architect robot systems with AI coding assistants. Write specifications, let AI generate implementation, validate results."
            />
            
            <FeatureCard
              icon="✨"
              title="100% Open Source"
              description="Free forever. No paywalls, no subscriptions. All code examples, simulation files, and robot models included on GitHub."
            />
            
            <FeatureCard
              icon="💰"
              title="Flexible Hardware Options"
              description="Start with free simulation. Progress to $200 Jetson setup. Scale to full humanoid hardware ($5k+). Learn at your budget level."
              badge="MOST VALUABLE"
            />
          </div>
        </section>

        {/* Course Overview Section */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>Complete 7-Part Curriculum</h2>
          <p className={styles.sectionSubtitle}>
            A structured journey from fundamentals to building advanced autonomous humanoid robots
          </p>
          
          <div className={styles.coursePartsGrid}>
            <CoursePartCard
              number="1"
              title="Foundations"
              description="Introduction to Physical AI, robotics fundamentals, and AI review"
              weeks="Week 1"
              linkUrl={useBaseUrl('/part-01-foundations/')}
            />
            
            <CoursePartCard
              number="2"
              title="ROS 2 Ecosystem"
              description="Master Robot Operating System 2 architecture and development"
              weeks="Weeks 2-3"
              linkUrl={useBaseUrl('/part-02-ros2-ecosystem/')}
            />
            
            <CoursePartCard
              number="3"
              title="Simulation Environments"
              description="Learn Gazebo and Isaac Sim for robot testing and validation"
              weeks="Weeks 4-5"
              linkUrl={useBaseUrl('/part-03-simulation-environments/')}
            />
            
            <CoursePartCard
              number="4"
              title="NVIDIA Isaac Platform"
              description="Deep dive into Isaac Sim, ROS integration, and AI workflows"
              weeks="Weeks 6-7"
              linkUrl={useBaseUrl('/part-04-nvidia-isaac-platform/')}
            />
            
            <CoursePartCard
              number="5"
              title="Humanoid Development"
              description="Build complete humanoid robots with manipulation and locomotion"
              weeks="Weeks 8-10"
              linkUrl={useBaseUrl('/part-05-humanoid-development/')}
            />
            
            <CoursePartCard
              number="6"
              title="Conversational Robotics"
              description="Integrate LLMs, voice interfaces, and multimodal AI"
              weeks="Week 11"
              linkUrl={useBaseUrl('/part-06-conversational-robotics/')}
            />
            
            <CoursePartCard
              number="7"
              title="Capstone Project"
              description="Build and deploy a complete autonomous humanoid system"
              weeks="Weeks 12-13"
              linkUrl={useBaseUrl('/part-07-capstone-project/')}
            />
          </div>
        </section>

        {/* Maturity Levels */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>Robotics Adoption Maturity Levels</h2>
          <p className={styles.sectionSubtitle}>
            Organizations and individuals progress through five stages of robotics capability. This textbook accelerates your journey from Level 1 to Level 4.
          </p>
          
          <div className={styles.maturityLevels}>
            <MaturityLevel
              level="1"
              title="Learning & Exploration"
              approach="Individual experimentation"
              description="Following tutorials, running example code, understanding ROS 2 basics"
              focus="START HERE"
              productivity="Building foundation"
            />
            
            <MaturityLevel
              level="2"
              title="Simulation Mastery"
              approach="Gazebo & Isaac Sim proficiency"
              description="Testing robot designs in virtual environments, validating before hardware"
              focus="WEEK 3-5"
              productivity="10x faster iteration"
            />
            
            <MaturityLevel
              level="3"
              title="Hardware Integration"
              approach="Real robot deployment"
              description="Deploying to physical robots, sensor calibration, real-world testing"
              focus="WEEK 8+"
              productivity="Production-ready skills"
            />
            
            <MaturityLevel
              level="4"
              title="Autonomous Systems"
              approach="Full autonomy stack"
              description="Complete SLAM + Navigation + Manipulation + Locomotion integration"
              focus="WEEK 13"
              productivity="Industry-level capability"
              isTarget={true}
            />
            
            <MaturityLevel
              level="5"
              title="Embodied AI Research"
              approach="Novel architectures"
              description="Creating new learning algorithms, multi-agent coordination, transfer learning"
              focus="BEYOND THIS BOOK"
              productivity="Research frontier"
            />
          </div>
        </section>

        {/* Comparison Table */}
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>Traditional Robotics vs AI-Native Robotics</h2>
          
          <div className={styles.tableContainer}>
            <table className={styles.comparisonTable}>
              <thead>
                <tr>
                  <th>Aspect</th>
                  <th>Traditional Robotics</th>
                  <th>AI-Native Robotics</th>
                </tr>
              </thead>
              <tbody>
                <tr>
                  <td><strong>Development</strong></td>
                  <td>❌ Write every line of code manually</td>
                  <td>✅ Write specifications, AI generates implementation</td>
                </tr>
                <tr>
                  <td><strong>Control</strong></td>
                  <td>❌ Hardcoded behaviors and state machines</td>
                  <td>✅ Learned policies and adaptive behaviors</td>
                </tr>
                <tr>
                  <td><strong>Perception</strong></td>
                  <td>❌ Hand-engineered feature extraction</td>
                  <td>✅ Deep learning for vision and sensor fusion</td>
                </tr>
                <tr>
                  <td><strong>Planning</strong></td>
                  <td>❌ Pre-computed paths and trajectories</td>
                  <td>✅ Real-time replanning with world models</td>
                </tr>
                <tr>
                  <td><strong>Adaptation</strong></td>
                  <td>❌ Fails in novel situations</td>
                  <td>✅ Generalizes to new environments</td>
                </tr>
                <tr>
                  <td><strong>Debugging</strong></td>
                  <td>❌ Step through thousands of lines</td>
                  <td>✅ Refine specifications and constraints</td>
                </tr>
                <tr>
                  <td><strong>Speed</strong></td>
                  <td>❌ Weeks to implement new behaviors</td>
                  <td>✅ Days with 5-10x faster iteration</td>
                </tr>
                <tr>
                  <td><strong>Scalability</strong></td>
                  <td>❌ Each robot requires custom code</td>
                  <td>✅ Transfer learning across platforms</td>
                </tr>
              </tbody>
            </table>
          </div>
        </section>

        {/* Final CTA */}
        <section className={styles.ctaSection}>
          <h2>Begin Your Journey into Physical AI</h2>
          <p>
            The era of humanoid robots is here. Join thousands of learners mastering embodied intelligence with ROS 2, simulation, and AI-native development.
          </p>
          <Link className={styles.ctaLarge} to={introUrl}>
            Start with Chapter 1 →
          </Link>
        </section>

        {/* Footer */}
        <footer className={styles.footer}>
          <div className={styles.footerGrid}>
            <div className={styles.footerColumn}>
              <h4>LEARN</h4>
              <ul>
                <li><Link to={introUrl}>Start Your Journey</Link></li>
                <li><Link to={assessmentsUrl}>Full Curriculum</Link></li>
                <li><Link to={introUrl}>Week 1 Chapter 1</Link></li>
              </ul>
            </div>
            
            <div className={styles.footerColumn}>
              <h4>COMMUNITY</h4>
              <ul>
                <li><a href="https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook" target="_blank" rel="noopener noreferrer">GitHub Repository</a></li>
                <li><a href="https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/discussions" target="_blank" rel="noopener noreferrer">Discussions</a></li>
                <li><a href="https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/issues" target="_blank" rel="noopener noreferrer">Report Issues</a></li>
              </ul>
            </div>
            
            <div className={styles.footerColumn}>
              <h4>RESOURCES</h4>
              <ul>
                <li><Link to={hardwareUrl}>Hardware Guide</Link></li>
                <li><Link to={setupUrl}>Setup Instructions</Link></li>
                <li><Link to={troubleshootingUrl}>Troubleshooting</Link></li>
              </ul>
            </div>            <div className={styles.footerColumn}>
              <h4>ABOUT</h4>
              <ul>
                <li><a href="https://panaversity.org/" target="_blank" rel="noopener noreferrer">Panaversity</a></li>
                <li><a href="https://panaversity.org/#about" target="_blank" rel="noopener noreferrer">Our Mission</a></li>
                <li><a href="https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/blob/main/CONTRIBUTING.md" target="_blank" rel="noopener noreferrer">Contributing</a></li>
              </ul>
            </div>
          </div>
          
          <div className={styles.footerCopyright}>
            Copyright © 2025 Panaversity • Physical AI & Humanoid Robotics • Free & Open Source • MIT License
          </div>
        </footer>
      </main>
    </Layout>
  );
}
