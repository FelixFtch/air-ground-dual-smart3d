# 速度驱动的 OHZ 形状：从人身后穿行

日期：2026-10-04。状态：讨论结论，**代码尚未修改**。现行 OHZ 仍是 `perception/hazards.py` 里固定 1 秒扫掠的椭圆。

## 观察

有人从机器人前方横穿时，SMART3D 经常从人的前方（人要去的方向）绕。人继续往前走，又把刚选的路挡住，路径被一路往人前进的方向推，反而绕远。

## 原因

- 现行 OHZ：把人与机器人尺寸之和 ρ 沿速度方向扫 1 秒，中心前移半程；身后只留 ρ，前方多出 v×1 s。
- SMART 把 OHZ 当静态区域：节点落在里面就剪，边穿过就作废，直到下次刷新。它不知道人"什么时候在哪"。
- 机器人到交叉点通常要几秒。OHZ 前端刚碰到机器人路线时，从人前方绕最短，于是选前方；每次刷新 OHZ 再往前推一截，形成追着人绕。
- 原版 SMART（Shen 等，RA-L 2023，arXiv 2305.06487）的 OHZ 是以障碍当前位置为圆心的圆，r = v·T_OH + r_i + r_R，T_OH = 0.4 s，假设障碍朝向未知。

## 想法

1. 有速度估计的人，OHZ 变成前长后短的水滴；没有速度的人保持圆。形状只由速度估计决定，不另做置信度调整。
2. 水滴取人在有限加速度 a 下、[0, T] 内可能到达的位置：t 时刻圆心为 p + v·t，半径为 ρ + ½·a·t²。
   - 前端：ρ + v·T + ½·a·T²
   - 两侧：ρ + ½·a·T²
   - 身后：保持 ρ。t ≤ 2v/a 时，正在前进的人来不及回到身后（v = 1 m/s、a = 1.5 m/s² 时约 1.3 s）。
3. 只在 LRZ 内使用这个变体，T 取机器人穿过 LRZ 所需时间的量级。LRZ 外沿用现有做法。
4. 判断时把水滴近似成半径沿预测方向逐渐变大的胶囊，点和边都有闭式判断，开销与椭圆相当。
5. 人群里每个移动的人身后形成通道：同向人流的通道连成可以跟随的"车道"；横穿人流可以依次从人身后穿过。

## 成立条件与风险

- **大多数人要有速度**：没有速度的人按圆处理，身后没有通道。2026-10-04 空场景 100 s 录制对照 CARLA 真值：YOLO26m + 地图关联约 64% 的正确检测带速度，线上 1 Hz 的 SAM3 约 26%。
- **时间窗**：人走过后约 2v/a 秒内从其身后穿过有物理依据；更久则只是依赖"人一般不会突然回头"。
- **刹停余地**：机器人必须随时能停下。本栈的人体本身也在 LiDAR 体素里，执行前还有按实测几何的扫掠检查兜底。
- **密度**：缝隙小于机器人尺寸时没有通道，退化为冻结机器人问题（Trautman & Krause，IROS 2010；Trautman 等，IJRR 2015）。
- **人的反应**：靠近机器人时人会减速或侧移，两侧的 ½·a·t² 加宽不能省。
- **舒适度**：紧贴人身后安全但不舒服，可另设一个最小距离。

## 可选的进一步做法（横穿场景）

- 时空判断：树节点按代价换算到达时间，与人在该时刻的位置比较（SIPP，Phillips & Likhachev ICRA 2011；速度障碍，Fiorini & Shiller IJRR 1998）。
- 路径-速度分解：横穿的人在"沿路径距离-时间"图上是一条斜带，减速让行即从人身后穿过；迎面的人才改路径（Kant & Zucker IJRR 1986；Baidu Apollo EM Planner 的 ST 图）。

## 验证计划

- 场景：UE 中等密度的横穿人流，计划用 MetaHuman Crowd 添加人物。
- 对比三种做法：现行椭圆、速度水滴、水滴加调速让行。
- 指标：穿越时间、与人的最小距离、绕行长度、从人身后穿过的比例、重规划次数。
- MetaHuman 人物接入前先确认：
  - 是否带碰撞体。CARLA 的射线雷达靠碰撞求交，没有碰撞体就不会出现在 LiDAR 和 OctoMap 里，只剩相机这一路能看到。
  - 真值从哪里取。它们不是 CARLA walker，`world.get_actors().filter('walker.*')` 拿不到，需要从 UE 侧导出位置和速度。

## 参考

- Koschi, Pek, Beikirch, Althoff. Set-Based Prediction of Pedestrians in Urban Environments Considering Formalized Traffic Rules. ITSC 2018.
- Vassallo 等. How do walkers avoid a mobile robot crossing their way? Gait & Posture 2017（arXiv 1609.07955）；以及 2018 年后续。
- Kirby, Simmons, Forlizzi. COMPANION. RO-MAN 2009（非对称高斯个人空间，前宽后窄）。
- Mavrogiannis 等. Core Challenges of Social Robot Navigation: A Survey. ACM THRI 2023.
