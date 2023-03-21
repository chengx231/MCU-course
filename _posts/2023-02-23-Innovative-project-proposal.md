---
layout: post
title: Innovative Project Proposal
author: [Richard Kuo]
category: [Lecture]
tags: [jekyll, ai]
---

This homework is to propose an innovative project and describe the key features, list all Design Considerations and the required technologies, then draw the System Block Diagram.

---
## Homework Report Format
**Contents:**<br>


---
## 居家盲人導引系統

### 應用功能說明
1. 時刻偵測前方以及地面物體 
2. 紀錄路徑與建構室內地圖
3. 紀錄使用者資訊

### 設計考量與相關技術
**系統設計考量：**<br>
1. 操作方式:ESP32-CAM，手持式 or 掛式3D掃描器
2. 供電方式:MCU
3. 聯網方式:WiFi、藍芽

**所需相關技術：**
1. 圖像辨識
2. 亮度感測
3. 紅外線感測
4. 訊號處理
5. 3D scanner 建模

### 系統方塊圖
![](https://github.com/chengx231/MCU-course/blob/5e10ff8cf4f483806da729c9316e7d5a59eae11b/images/project.jpg?raw=true)


<br>
<br>

*This site was last updated {{ site.time | date: "%B %d, %Y" }}.*


