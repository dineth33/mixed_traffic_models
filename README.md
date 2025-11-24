# IAM — Mixed Traffic Model

This repository contains a **Python implementation** of the *Intelligent Agent Model (IAM)*, a mixed-traffic microscopic simulation model capable of predicting both **lateral** and **longitudinal** vehicle motion. 
IAM simulates how human drivers and autonomous agents interact in complex traffic environments, enabling realistic modelling of lane-changing, acceleration, car-following, and evasive maneuvers.

This Python version aims to provide a clean and modular translation of the original JavaScript code. 

The original IAM implementation was written in JavaScript and can be found here:  
👉 [MTGermany/mixedTraffic](https://github.com/MTGermany/mixedTraffic)

## Calibration Procedure

The calibration of the IAM can be performed by following the steps outlined in the [`IAM_calib.ipynb`](./IAM_calib.ipynb) notebook.

Because the IAM is built on a longitudinal driving model, the selected longitudinal model must be calibrated before calibrating the IAM itself. In our implementation,
we use the Intelligent Driver Model (IDM). The resources and scripts for IDM calibration are provided in the following file: [`IDM_calib.py`](./model_files/IDM_calib.py)

---

## Papers
Treiber, M., & Chaudhari, A. A. (2023). *The intelligent agent model — a fully two-dimensional microscopic traffic flow model.* arXiv.  
🔗 https://arxiv.org/abs/2310.16816

Kesting Arne, Treiber Martin and Helbing Dirk 2010Enhanced intelligent driver model to access the impact of driving strategies on traffic capacityPhil. Trans. R. Soc. A.3684585–4605
🔗  http://doi.org/10.1098/rsta.2010.0084



