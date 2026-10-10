# 人员 OHZ：给要改形状的人

这条链路现在是现场在用的。不要改 `planning/` 里的 RRT* 和 LRZ，除非形状接口本身不够用。

## 数据怎么走

1. `perception/yolo26_backend.py`：YOLO26m 只检测人，MapTracker 给每台相机一个编号。
2. `perception/people_projection.py`：掩码加深度，投成地图上的 3D 框，并在同一编号上估计水平速度。
3. `perception/hazards.py`：`build_hazards` 把人和这台机器人的半径收成禁区。Go2 是地面椭圆，Bee 是椭球。
4. `planning/src/people.rs`：只接受 `ellipse` / `ellipsoid`，用它剪树。人离开或到期后，同一条清除逻辑把树枝接回去。

2D 和 3D 已经按机器人的 `dimension` 分开。Go2 读地面椭圆，Bee 读椭球。

## 要换成的形状

人不动：不要拉长。禁区是人的半径加上这台机器人的半径。Go2 是地面上的圆，Bee 是同样半径、带高度的圆柱。

人在走：脚底按速度方向在地面上拉矩形。宽是人加机器人的直径，长度由速度决定，从脚底往前伸。这整块用来剪树。人停下后矩形消失，只留圆或圆柱。

矩形还没接上。`people.rs` 现在拒绝椭圆以外的形状。新形状要作为第二种输出留在 `perception/hazards.py`，并让 `people.rs` 认得它。椭圆先留着，方便对照。不要把人写进占据地图。
