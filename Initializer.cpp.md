## 标题:从理论到实践: ORB-SLAM3 Initializer完全解读

作者:Liam

博客:https://blog.csdn.net/qq_40114620/article/details/109799810#comments_13915791

### **构造函数**

Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
**参数:** 参考帧(第一帧), 误差, 迭代次数
**操作:**读取参考帧的相机模型, 内参, 去畸变的特征点等传入参数

### **初始化**:并行的计算前后两帧的本质矩阵和基础矩阵,选出来评分高的恢复旋转和平移

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
												vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
**参数:** 当前帧(第二帧), 前后帧的匹配关系(), 参考帧到当前帧的旋转, 参考帧到当前帧的平移(==当前帧指向参考帧==), 三角化后的点, 特征是否被三角化过
**返回值**：bool->初始化是否成功的标志
**操作**:

- vMatches12中的匹配关系以<==第一帧特征索引,第二帧特征索引==>存储在mvMatches12中, 同时mvbMatched1[i]设置为true表示第一帧中该索引的特征点匹配成功
- 从匹配中不重复的随机选择mMaxIterations组点,每组8个
- 开两个线程同时计算单应和基本矩阵
- 计算得分
- 选择得分高的矩阵来恢复两帧位姿

### **寻找最优单应矩阵**

void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
**参数**: 匹配的Inliers, 最后的得分, 单应矩阵
**操作**:

- 归一化
- 利用选择的mMaxIterations组匹配点用八点法计算单应矩阵（ComputeH21(vPn1i,vPn2i)）
- 恢复初始尺度（归一化前的）
- 保留最高得分的单应矩阵及对应的匹配内点

### **需要最优基础矩阵**

void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
**参数**: 匹配的Inliers, 最后的得分, 基础矩阵
**操作**:

- 归一化
- 利用选择的mMaxIterations组匹配点用八点法计算基础矩阵（ComputeF21(vPn1i,vPn2i)）
- 恢复初始尺度（归一化前的）
- 保留最高得分的基础矩阵及对应的匹配内点

### **计算单应矩阵**

cv::Mat Initializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
**参数：**mMaxIterations中的一组匹配点vP1和vP2
**返回值：**利用两帧匹配点计算出来的单应矩阵
**理论推导**：
平面P的方程：$n^TP+d=0$，很容易知道$-n^TP/d = 1$;
对于两帧的匹配点$p_1，p_2$，有
$$
p_2 \backsimeq K(RP+t)\\
\backsimeq K(RP+t(-n^TP/d))\\
\backsimeq K(R - n^TP/d)P\\
\backsimeq K(R - n^TP/d)K^{-1}p_1
$$
令$H=K(R - n^TP/d)K^{-1}$，所以$p_2=Hp_1$。
把H矩阵的形式设出来为
$$
H = \left[ \begin{matrix}h_1 & h_2 & h_3 \\ h_4 & h_5 & h_6 \\ h_7 & h_8 & h_9 \end{matrix} \right]
$$
所以：
$$
\left[\begin{matrix} u_2 \\ v_2 \\ 1\end{matrix} \right] = \left[ \begin{matrix}h_1 & h_2 & h_3 \\ h_4 & h_5 & h_6 \\ h_7 & h_8 & h_9 \end{matrix} \right]\left[\begin{matrix} u_1 \\ v_1 \\ 1\end{matrix} \right]
$$
整理矩阵可以得到：
$$
u_2 = \frac{h_1u_1+h_2v_1+h_3}{h_7u_1+h_8v_1+h_9} \\
v_2 = \frac{h_4u_1+h_5v_1+h_6}{h_7u_1+h_8v_1+h_9}
$$
H矩阵没有尺度，所以任意乘以或者除以一个非零常数保持不变，所以这里我们直接令$h_9=1$，整理可以得到：
$$
{h_1u_1+h_2v_1+h_3}-{h_7u_1u_2-h_8v_1u_2}-u_2=0 \\
{h_4u_1+h_5v_1+h_6}-{h_7u_1u_2-h_8v_1u_2}-v_2=0
$$
这样就可以用一组匹配点构造两个约束(其实是三个，但是三个线性相关，所以只取前两个)，所以自由度为8的单应矩阵需要四组匹配点就可以算出。
**操作：**

- 利用八组匹配点构造A矩阵
- 利用SVD分解求解AX=0型的方程
- 最小特征值对应的特征向量就是方程的解（可参考多视图几何第二版198-200页）

### **计算基础矩阵**

cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
**参数：**mMaxIterations中的一组匹配点vP1和vP2
**返回值：**利用两帧匹配点计算出来的基础矩阵
**理论推导**：
第一帧下空间点为$P=[X,Y,Z]$
根据针孔模型得到像素点的位置：
$$
s_1p_1=KP \\
s_2p_2=K(RP+t)
$$
写成齐次坐标的形式$sp \backsimeq p$
这时候可以写成：
$$
p_1 \backsimeq KP, p_2  \backsimeq K(RP+t)
$$
归一化平面（z=1）上的点可以表示为：
$$
x_1=K^{-1}p_1, x_2=K^{-1}p_2
$$
所以：
$$
x_2  \backsimeq Rx_1+t
$$
等式两边同时左乘t^，再左乘$x_2^T$
$$
x_2^Tt^{\wedge}x_2 \backsimeq x_2^Tt^{\wedge}Rx_1
$$
易知$x_2^Tt^{\wedge}x_2=0$，所以：
$$
x_2^Tt^{\wedge}Rx_1=0
$$
代入像素点$p_1,p_2$
$$
p_2^TK^{-T}t^{\wedge}RK^{-1}p_1=0
$$
令$F=K^{-T}t^{\wedge}RK^{-1}$，所以$p_2^TFp_1=0$。以下的推导和H矩阵的推导相同，每对匹配点可以得到一个约束，由于F矩阵的行列式为切具有齐次性，所以自由度为7，最少利用七组点可以求出F矩阵。
**操作：**

- 利用八组匹配点构造A矩阵
- 利用SVD分解求解AX=0型的方程
- 把求出来的矩阵再进行SVD分解并置最小的特征值为0
- 利用计算出来的U和$V^T$恢复F矩阵

### **检验单应矩阵**(在FindHomography中调用)

float Initializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
**参数：**从1到2的单应矩阵H21，从2到1的单应矩阵H12，匹配的内点vbMatchesInliers，测量误差sigma
**返回值：**该单应矩阵的得分
**操作：**

- 分别取出H12和H21中的所有的元素
- 设置卡方分布为自由度为2（单目），误差项有百分之九十五不符合高斯分布的阈值为5.991(高斯分布的平方和就是卡方分布，重投影误差满足高斯分布)。具体可以参考https://www.cnblogs.com/hardjet/p/11708202.html
- 对于两帧的所有Inlier匹配点：
          提取两个特征点的u, v值
          利用H矩阵把第二帧（第一帧）的点投影到第一帧（第二帧）   
          计算投影点和匹配点的距离（重投影误差）
        		距离乘上不同高斯金字塔层的缩放系数得到chiSquare1
        		如果距离大于阈值则直接跳过
          小于阈值利用score += th - chiSquare1计算得分
          最后返回总得分

### **检验本质矩阵**(在FindFundamental中调用)

float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
**参数：**从1到2的本质矩阵F21，匹配的内点vbMatchesInliers，测量误差sigma
**返回值：**该本质矩阵的得分
**操作：**

- 分别取出F21中的所有的元素
- 设置卡方分布为自由度为1（单目），误差项有百分之九十五不符合高斯分布的阈值为3.841（这里的重投影误差计算的是点到线的距离）；同时保留了thScore = 5.991，这样计算出来的得分才可以和单应矩阵计算出来的得分比较。（单应矩阵是点到点的形式，基础矩阵是点到线的形式）
- 对于两帧的所有Inlier匹配点：
          提取两个特征点的u, v值
          利用H矩阵把第二帧（第一帧）的点投影到第一帧（第二帧）的基线上   
          计算匹配点到基线的距离（重投影误差）
        		距离乘上不同高斯金字塔层的缩放系数得到chiSquare1
        		如果距离大于阈值则直接跳过
          小于阈值利用score += thScore - chiSquare1计算得分
          最后返回总得分

###  **利用F矩阵恢复R,t**

bool Initializer::ReconstructF
**参数：**匹配的Inliers，基础矩阵F21，相机内参K，旋转R21，平移t21，恢复的3D点vP3D，是否被三角化vbTriangulated，三角化测量有效的最小视差角minParallax，使用三角化测量进行数据判断的最小测量点数量minTriangulated
**操作：**

- 利用基础矩阵计算本质矩阵E
- 利用函数DecomposeE从E矩阵中恢复R，t
- 利用函数CheckRT检验四组R，t得到每组解下的GOOD点
- 把nGood最大的赋值为maxGood作为一个判断条件，选择特征点数量的0.9倍和三角化的特征点作为nMinGood作为最低阈值。
- 如果CheckRT返回的nGood大于0.7×maxGood就认为这组解是一起潜在的正确解，nsimilar++。
- 如果判断出nsimilar>1或者maxGood<nMinGood则认为计算失败
- 否则判断maxGood是哪组解产生的，如果这组的视差也大于最小视差阈值，就把该组的R，t作为正确解。

###  **利用H矩阵恢复R,t**

bool Initializer::ReconstructH
**参数：**匹配的Inliers，单应矩阵H21，相机内参K，旋转R21，平移t21，恢复的3D点vP3D，是否被三角化vbTriangulated，三角化测量有效的最小视差角minParallax，使用三角化测量进行数据判断的最小测量点数量minTriangulated
**推导：（实在是不想打公式了）**

![image.png](https://i.loli.net/2020/11/13/eTuEYHs4SbRMB7K.png)

**操作：**

- 根据前边的推导$G=K^{-1}HK$，得到了像素平面的对应关系
- 对A进行SVD分解，其中w矩阵为单应矩阵，特征值分别为d1,d2,d3。
- **当d'=d2时**
- 分别计算法向量n的分量aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3))，x2=0，x3=sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3))
- 根据推导x1,x3可正可负，所以列出所有的情况x1[]，x3[]
- 分别计算旋转矩阵sin和cos，并列出所有的情况stheta[]
- 根据不同的Rp得到不同的旋转矩阵存到vR中
- 根据前边的推导带入得到平移量tp，根据变换关系得到真正的t并归一化，把所有的情况存到vt中
- 把所有的方向向量存到vn中并保证方向向量z值大于0
- **当d'=-d2时**，所有的推导过程基本一致。
- 最后根据得到不同的解利用CheckRT进行检验得到正确的解。

###  **特征点三角化**

void Initializer::Triangulate
**参数：**特征点kp1和特征点kp2，投影矩阵P1和P2，三角化出来的点的坐标x3D
**推导：**对于两个归一化的特征点，他们其实表示的是空间中的同一个点，所以：
由投影方程易知
$$
x = sPX \\
sx^{\wedge}PX=0\\
$$
所以带入具体的表达式可以得到：
$$
\left[\begin{matrix}u \\v \\ 1 \end{matrix}\right]=\left[\begin{matrix}P_0 \\ P_1 \\ P_2 \end{matrix}\right]\left[\begin{matrix}X\\Y\\Z\\1\end{matrix}\right],P_0 = [p_0,p_1,p_2,p_3] \\
x^{\wedge}=\left[\begin{matrix}0 & -1 & v \\1 & 0 & -u \\ -v & u & 0\end{matrix}\right]\\
x^{\wedge}PX=\left[\begin{matrix}0 & -1 & v \\1 & 0 & -u \\ -v & u & 0\end{matrix}\right]\left[\begin{matrix}P_0 \\ P_1 \\ P_2\end{matrix}\right]
$$
这样可以得到：
$$
\left[\begin{matrix}-P_1+vP_2\\P_0-uP_2\\-vP_0+uP_1\end{matrix}\right]X=0
$$
又因为：
$$
-vP_0+uP_1=-u(-P_1+vP_2)-v(P_0-uP_2)
$$
所以系数矩阵的秩为2，只取前两行并整理，对于两个点可以写成：
$$
\left[\begin{matrix}-P_1+v_1P_2\\P_0-u_1P_2\\-P_2+v_2P_2\\P_0-u_2P_2\end{matrix}\right]X=0
$$
**操作：**

- 设置A矩阵构造为最后我们求出的形式
- 直接对A矩阵进行SVD分解
- ||AX||min = X'A'AX = X'lamdarX = lamdar||X||=lamdar --> lamdar就是A'A的特征值
- SVD得到的V'的最后一行就是最小特征值对应的特征向量，就是方程的解。
- 归一化输出3D点

### **归一化尺度**

void Initializer::Normalize
**参数：**特征点集合vKeys，归一化后的点集合vNormalizedPoints，特征点归一化矩阵T
**操作：**

- 遍历所有的特征点分别累加X，Y
- 利用累加值和特征点数量计算X，Y的均值
- 把原始特征点的坐标值都减去均值使其在X轴和Y轴上的均值为0，并把减去均值后的值的绝对值相加得到点偏离X，Y轴的程度。
- 取偏离值的和除以特征点个数得到平均每个点的偏离值，把该值的倒数作为尺度因子
- 遍历所有的去均值后的特征点乘以该尺度因子得到归一化后的特征点（这样使x,y方向的一阶矩为1完成了归一化）
- 记录该归一化系数到T矩阵

### **检查解算出的R，T**

int Initializer::CheckRT
**参数：**待验证的旋转矩阵和平移向量R，t；当前帧和参考帧的特征点vKeys2，vKeys1；两帧的匹配关系vMatches12和Inliers的匹配关系vbMatchesInliers；相机内参K；三角化后的点vP3D；重投影误差阈值th2；特征点对中good点标记vbGood；计算出来比较大的视差parallax
**操作：**

- 读取并计算第一帧的投影矩阵和第二帧的投影矩阵并设置第一帧的光心为原点，第二帧的光心为-R.t()*t
  $$
  O_1=RO_2+t,O_1=\left[\begin{matrix} 0 \\ 0\\ 0\end{matrix}\right]
  $$
  所以：
  $$
  O_2=-R^T*t
  $$

- 遍历所有匹配点并跳过outliers，利用上边介绍的特征点三角化函数得到三角化后的特征点

- **判断三角化后的特征点各个值是不是正常的std::isfinite()**

- 如果均正常则把该3D点分别减去两个相机光心得到两个向量

- 利用两个向量点乘再除以模长得到两个向量的夹角的cos值cosParallax

- **如果这个3D点的z值为负且视差太小就丢弃该对匹配（参考帧）**

- **把该3D点利用旋转矩阵和平移向量投影到第二帧判断z值和视差考虑是否丢弃（当前帧）**

- **分别把该3D点投影到当前帧和参考帧计算重投影误差判断是否丢弃该对匹配**

- 如果经历住了上边的考验就把特征点的视差和在第一帧下的3D点存储起来，nGood++

- 把好点中的视差排序，如果好点小于50个就取最后一对的视差作为返回值，大于50个时选择第50个作为返回值

### **分解E矩阵**(证明参考多视图几何第二版P199-P200)

本质矩阵有五个自由度（旋转平移各三个，去除一个尺度因子，所以本质矩阵也是一个齐次量）
$$
E=t^{\wedge}R
$$
void Initializer::DecomposeE
**参数：**本质矩阵E；旋转矩阵R1和R2,当前帧到参考帧的平移t
**操作：**

- 对本质矩阵进行SVD分解
- **左奇异值（AA'）的最后一列就是t**
- 对t进行归一化
- 构造W矩阵
- 利用W矩阵得到两个旋转矩阵
- 最后分解E矩阵可以得到四组解利用CheckRT验证。





