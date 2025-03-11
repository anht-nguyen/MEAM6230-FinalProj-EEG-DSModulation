# Final Project: EEG-Based Adaptive Humanoid Robot for Action Observation Therapy 

## Proposal Abstract:

This project explores the real-time modulation of a humanoid robot’s action speed using EEG-based engagement measures to enhance Action Observation (AO) Therapy. Inspired by Oliver et al. (2024), EEG biomarkers such as relative alpha power and theta-alpha ratios will be used to quantify user engagement. These neural markers will dynamically adjust the movement speed of a humanoid robot with two arms (each 4DOFs) performing predefined therapeutic actions. The adaptation mechanism will be based on dynamical system modulation techniques, following approaches used by Iwane et al. (2023) and Batzianoulis et al. (2021), allowing smooth and continuous adjustment to optimize engagement. By integrating real-time EEG analysis and robotic adaptability, this study aims to improve user participation in rehabilitation, potentially leading to more effective motor learning outcomes.

**Keywords**: 
EEG, Engagement, Action Observation Therapy, Humanoid Robot, Dynamical Systems, Rehabilitation Robotics

**Significance**: 
By integrating real-time brain activity monitoring with robotic adaptation, the proposed system aims to address a critical challenge in robotic rehabilitation—sustaining engagement over extended therapy sessions. This adaptive system could improve motor learning outcomes by maintaining high levels of cognitive participation, leading to enhanced neuroplasticity.



---

## **Project Objectives**
1. **Develop an EEG-Based Engagement Model**  
   - Implement real-time EEG signal processing to extract engagement markers such as relative alpha power and theta-alpha ratio.  
   - Validate engagement metrics based on previous research on EEG-based human-robot interaction.  

2. **Integrate EEG with a Humanoid Robot for Adaptive Therapy**  
   - Implement a closed-loop system where the robot’s action speed is modulated based on real-time EEG engagement measures.  
   - Ensure smooth and continuous adaptation using dynamical system modulation techniques.  

3. **Evaluate the Effectiveness of Engagement-Based Adaptation**  
   - Conduct an experimental study to compare engagement levels between static-speed and adaptive-speed robotic action conditions.  
   - Assess the impact of adaptive engagement modulation on participants’ sustained attention and motor learning potential.  

---

## **Deliverables**
1. **EEG Engagement Measurement System**  
   - A real-time EEG processing pipeline capable of extracting and analyzing engagement-related biomarkers.  

2. **Adaptive Control Framework for the Humanoid Robot**  
   - A speed-modulation algorithm implemented using a dynamical system approach to adjust robot movements.  

3. **Experimental Protocol for AO Therapy Study**  
   - A structured experimental design to evaluate the system’s effectiveness in enhancing engagement.  

4. **Performance Evaluation and Analysis Report**  
   - Statistical analysis comparing engagement levels across different experimental conditions.  
   - Insights on the feasibility of real-time EEG-driven robotic adaptation in rehabilitation settings.  

---

## **Experimental Setup**
### **Participants:**  
- Healthy adult participants (N = 2-3) with no neurological impairments.  

### **EEG System:**  
- 32-channel EEG (Emotiv EPOC Flex, EasyCap M1)  
- Sampling rate: ?? Hz  
- Electrodes placed according to the 10-20 international system  

### **Humanoid Robot:**  
- A humanoid robot with two 4-degree-of-freedom (DOF) arms  
- Predefined action sequences related to AO therapy (intransitive actions)  

### **Procedure:**  
1. **Baseline Task:**  
   - Participants observe robot actions at a fixed speed while EEG data is recorded.  
   - Engagement is measured using power spectral density analysis.  

2. **Adaptive Task:**  
   - The robot dynamically adjusts its speed based on real-time EEG engagement levels.  
   - If engagement decreases (higher alpha power, lower theta-alpha ratio), speed is increased or decreased accordingly.  

3. **Post-Experiment Assessment:**  
   - Participants complete subjective engagement questionnaires.  
   - EEG engagement trends are analyzed statistically.  

---

## **Expected Outcomes**
- **Improved Engagement Maintenance:** Adaptive speed modulation is expected to sustain higher engagement levels compared to fixed-speed conditions.  
- **Better Understanding of EEG-Based Engagement Markers:** The study will validate EEG metrics as reliable indicators of cognitive engagement during AO therapy.  
- **Potential for Personalized Rehabilitation Systems:** Findings could contribute to future personalized robotic therapy applications for stroke and motor-impaired patients.  

---


## References


### EEG engagement

https://arxiv.org/abs/2411.18587

### EEG-based DS Modulation

https://www.nature.com/articles/s41598-023-47136-2

https://www.nature.com/articles/s42003-021-02891-8