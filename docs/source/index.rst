ENPM818Z — On-Road Automated Vehicles
=====================================

Course Description
------------------
ENPM818Z provides a deep dive into the core technical and technological components of automated passenger vehicles for on-road applications. Students explore the essential systems that enable self-driving capabilities, including perception, sensor fusion, localization, motion planning, and control.  

The course emphasizes a hands-on approach using the **CARLA** simulation environment, where students develop and test advanced driving algorithms in simulated urban and highway scenarios. Core topics include:

- Multi-sensor perception (LiDAR, RADAR, cameras, IMU, GNSS)
- Real-time data fusion and SLAM-based localization
- Motion planning and trajectory optimization
- AI-driven decision-making for behavior prediction and control
- System integration and simulation-based validation

By the end of the semester, students will have designed, implemented, and tested components of an **automated driving system (ADS)**, gaining the technical foundation required for careers in robotics, automated vehicle engineering, and intelligent transportation systems.

Prerequisites
--------------
Students enrolling in ENPM818Z must have:

- ENPM673 (Perception for Autonomous Robotics) or equivalent  
- Proficiency in **ROS 2** for developing and testing robotic systems  
- Strong programming skills in **Python**  
- Basic understanding of robotics and computer vision  
- Familiarity with simulation environments such as CARLA (recommended)

Learning Outcomes
-----------------
Upon successful completion of this course, students will be able to:

- **Understand Core AV Technologies:** Explain how perception, localization, motion planning, and control interact within an ADS.  
- **Implement Multi-Sensor Fusion:** Combine data from LiDAR, RADAR, camera, IMU, and GNSS sensors to improve perception accuracy.  
- **Develop Localization and Mapping Systems:** Implement SLAM and evaluate localization accuracy under dynamic conditions.  
- **Apply Motion Planning Techniques:** Create safe and efficient motion planners and controllers for urban and highway driving.  
- **Design AI-Driven Decision Systems:** Apply machine learning or rule-based methods for decision-making in traffic scenarios.  
- **Integrate and Validate AV Systems:** Use CARLA simulation to integrate multiple modules into a working ADS pipeline.  
- **Analyze System Performance:** Evaluate robustness and safety using simulation metrics and performance indicators.

Course Resources
----------------
**Required Software and Tools**
- Ubuntu 22.04 LTS or 24.04 LTS  
- CARLA Simulator 0.9.15  
- ROS 2 (Humble or Jazzy)  
- Python 3.8+ with `numpy`, `matplotlib`, `opencv-python`, and `carla` packages  
- Visual Studio Code or preferred IDE  
- Git and GitHub for version control  

**Hardware Recommendations**
- GPU: NVIDIA GTX 1060 (1070+ recommended)  
- RAM: 8 GB minimum (16 GB+ preferred)  
- CPU: Quad-core processor  
- 20 GB free storage for CARLA and datasets  

Course Structure
----------------
ENPM818Z combines lectures with intensive, hands-on programming sessions in CARLA. Each week builds on prior material — progressing from single-sensor processing to full system integration. Students complete a sequence of assignments leading to a **final project** implementing a functional ADS pipeline.

Assignments and Evaluation
--------------------------
- **Assignments (30%)** – Four hands-on exercises covering sensing, fusion, localization, and planning.  
- **Quizzes (20%)** – Short in-class quizzes reinforcing core concepts.  
- **Final Project (50%)** – A complete ROS 2 ADS implementation with written report and final presentation.  

Late submissions incur a 10% deduction per day (maximum 3 days). Beyond 3 days, submissions receive zero credit.

.. toctree::
   :maxdepth: 2
   :titlesonly:

   assignments
   lectures

