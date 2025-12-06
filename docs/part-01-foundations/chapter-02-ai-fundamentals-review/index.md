---
sidebar_position: 0
---

# Chapter 2: AI Fundamentals Review

Welcome to Chapter 2! Whether you're coming from a software engineering background, have AI/ML experience, or are starting fresh, this chapter ensures everyone has the necessary artificial intelligence foundations for robotics applications.

## Why Review AI Fundamentals?

Physical AI sits at the intersection of **classical robotics** and **modern machine learning**. While robotics has decades of theory (kinematics, dynamics, control), the AI revolution has transformed how robots:

- **Perceive** their environment (computer vision, depth estimation, object detection)
- **Plan** their actions (reinforcement learning, trajectory optimization)
- **Interact** with humans (natural language processing, vision-language models)
- **Learn** from experience (imitation learning, sim-to-real transfer)

This chapter provides a focused review of AI concepts that directly enable Physical AI systems. We skip pure theory and emphasize **robotics-relevant applications**.

## What You'll Learn

### 1. [Machine Learning Basics](./01-machine-learning-basics.md)
Understand supervised, unsupervised, and reinforcement learning paradigms. Learn about overfitting, cross-validation, and evaluation metrics—contextualized for robot learning scenarios.

### 2. [Neural Networks Refresher](./02-neural-networks-refresher.md)
Review neural network architectures, backpropagation, activation functions, and optimization. Explore CNNs for vision, RNNs for sequences, and Transformers for modern perception.

### 3. [Computer Vision Fundamentals](./03-computer-vision-fundamentals.md)
Deep dive into how robots "see": image processing, feature extraction, object detection (YOLO, Faster R-CNN), semantic segmentation, and depth estimation.

### 4. [Natural Language Processing Basics](./04-nlp-basics.md)
Understand how robots process language: tokenization, embeddings (Word2Vec, BERT), intent detection, and integration with modern LLMs for conversational robotics.

### 5. [Reinforcement Learning Introduction](./05-reinforcement-learning-intro.md)
Learn how robots learn from trial and error: Markov Decision Processes, Q-learning, policy gradients, reward shaping, and sim-to-real transfer challenges.

## Prerequisites

- **Basic Python** programming (we'll show code examples)
- **Linear algebra** comfort (vectors, matrices, matrix multiplication)
- **Calculus basics** (derivatives, gradients—we'll review when needed)
- **Curiosity** about how AI enables robot intelligence

*If some concepts feel unfamiliar, don't worry—we build from fundamentals and provide additional resources.*

## Estimated Time

**⏱️ Total Time**: 8-12 hours to complete all lessons

- **Core Lessons**: 6-8 hours (5 lessons × 1.5 hours average)
- **Interactive Exercises**: 2-3 hours (coding exercises, visualization)
- **Further Reading**: 1-2 hours (research papers, tutorials)

## Learning Approach

This chapter is **more technical** than Chapter 1. You'll see:

- **Mathematical formulations** (with intuitive explanations)
- **Python code examples** (TensorFlow/PyTorch, NumPy)
- **Visualization exercises** (plot decision boundaries, training curves)
- **Robotics-contextualized problems** (not generic ML tutorials)

**Study Tips:**
1. **Run code examples** — Don't just read; execute and modify code
2. **Visualize concepts** — Draw diagrams, plot functions, sketch architectures
3. **Connect to robotics** — Ask "How does this enable robot behavior?"
4. **Review as needed** — Reference lessons when you encounter these concepts later in ROS 2/Isaac Sim

## How This Supports Physical AI

Every lesson connects directly to robotics applications:

- **ML Basics** → Training robot perception models, evaluating control policies
- **Neural Networks** → Powering vision systems, trajectory prediction, grasping
- **Computer Vision** → Enabling navigation, object manipulation, human detection
- **NLP** → Conversational robots, voice commands, instruction following
- **Reinforcement Learning** → Learning locomotion, manipulation skills, autonomous decision-making

By the end of this chapter, you'll understand the AI techniques that transform static robots into adaptive, learning-capable systems.

---

**Ready to begin?** Start with [Lesson 1: Machine Learning Basics](./01-machine-learning-basics.md)
