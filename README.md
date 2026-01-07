# 🤖 ROS2 기반 자율주행 로봇 통합 관제 플랫폼

> **"웹에서 클릭 한 번으로, 로봇이 현실 세계를 움직입니다."**
> 사용자가 웹으로 물품을 주문하면, 자율주행 로봇이 스스로 경로를 찾아 배달하는 **O2O(Online to Offline) 통합 관제 시스템**입니다.

<br>

## 🛠 기술 스택 (Tech Stack)

### 🎨 Frontend
![React](https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB)
![JavaScript](https://img.shields.io/badge/JavaScript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black)
![HTML5](https://img.shields.io/badge/HTML5-E34F26?style=for-the-badge&logo=html5&logoColor=white)
![CSS3](https://img.shields.io/badge/CSS3-1572B6?style=for-the-badge&logo=css3&logoColor=white)

### ⚡ Backend & Database
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![FastAPI](https://img.shields.io/badge/FastAPI-009688?style=for-the-badge&logo=fastapi&logoColor=white)
![MySQL](https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=mysql&logoColor=white)

### 🤖 Robotics & OS
![ROS2](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

<br>

## 🏛 시스템 아키텍처 (Architecture)

```mermaid
graph LR
    A[🧑‍💻 사용자] -->|주문 클릭| B(⚛️ React 웹)
    B -->|REST API| C{⚡ FastAPI 서버}
    C -->|데이터 저장| D[(🐬 DB)]
    C <-->|Topic/Action| E[🐢 ROS2 Node]
    E <-->|제어 명령| F[🤖 터틀봇]
    
    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#61DAFB,stroke:#333,stroke-width:2px
    style C fill:#009688,stroke:#333,stroke-width:2px
    style E fill:#22314E,stroke:#fff,stroke-width:2px,color:#fff
