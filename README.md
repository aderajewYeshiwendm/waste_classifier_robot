# Waste Classification Robot

This project implements a robotic system for waste classification using a Convolutional Neural Network (CNN). The robot is simulated in Gazebo with ROS 2 and uses a camera to capture images of waste objects. The CNN model classifies the waste into categories such as organic and recyclable. Then the robot picks the object and places it into the organic bin or recyclable bin


## Project Overview

The goal of this project is to classify waste materials using a robotic system. The system consists of:
- A **camera** to capture images of waste objects.
- A **CNN model** trained to classify waste into categories (e.g., organic, recyclable).
- A **robotic arm** simulated in Gazebo.
- **ROS 2** for communication and control.


## Dataset

The dataset used for training the CNN model is the **Waste Classification Data** from Kaggle:
- [Waste Classification Data on Kaggle](https://www.kaggle.com/datasets/techsash/waste-classification-data?resource=download)
- The dataset contains two main categories: `O` (Organic) and `R` (Recyclable).
- Images are organized into `TRAIN` and `TEST` directories.

### Prerequisites
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Python 3.8+**
- **TensorFlow/Keras**
- **Gazebo**
