"""
homework题目一
"""

import numpy as np
<<<<<<< HEAD
import pandas as pd
import matplotlib.pyplot as plt

# 四元数向量形式乘法
# p=[pw,px,py,pz].T
def quaternion_multiply(p, q):
    pw = p[0,0]
    qw = q[0,0]
    pv = p[1:4,0]
    qv = q[1:4,0]
    # quaternion_new = np.array([[pw*qw-pv.T,qv],
    #                   [pw*qv+qw*pv+np.cross(pv,qv)]])
    scalar = np.array(pw * qw - np.dot(pv.T, qv))  # 修正点积计算
    vector = np.array(pw * qv + qw * pv + np.cross(pv, qv, axis=0)).reshape(-1,1)

    quaternion_new = np.vstack([scalar,vector])

    return quaternion_new



# 将R（B->D）转换为四元数
def calculate_from_R_to_quaternion(t):
    omega = 0.5
    wt = omega*t
    a = np.pi/12
    R_BD = np.array([[np.cos(wt), -np.sin(wt)*np.cos(a), np.sin(wt)*np.sin(a)],
            [np.sin(wt), np.cos(wt)*np.cos(a), -np.cos(wt)*np.sin(a)],
            [0,          np.sin(a),             np.cos(a)]])

    trace = R_BD[0, 0] + R_BD[1, 1] + R_BD[2, 2]
    qw = np.sqrt(1+trace)/2
    qx = (R_BD[2, 1]-R_BD[1, 2])/(4*qw)
    qy = (R_BD[0, 2]-R_BD[2, 0])/(4*qw)
    qz = (R_BD[1, 0]-R_BD[0, 1])/(4*qw)
    q = np.array([[qw],[qx],[qy],[qz]])
    return q



# 读取tracking.csv中的无人机轨迹 shape=(860,5)
df = pd.read_csv("documents/tracking.csv")
t_sequence = np.array(df["t"]).reshape(1,-1)
qw_sequence = np.array(df["qw"]).reshape(1,-1)
qx_sequence = np.array(df["qx"]).reshape(1,-1)
qy_sequence = np.array(df["qy"]).reshape(1,-1)
qz_sequence = np.array(df["qz"]).reshape(1,-1)
# print(qw_sequence.shape)          (1,860)
q = np.array([[qw_sequence],[qx_sequence],[qy_sequence],[qz_sequence]]).reshape(4,-1)
# print(q.shape)                    (4,860)
length_sequence = t_sequence.shape[1]


q_answer_sequence = np.zeros((4,length_sequence))
for i in range(0,length_sequence):
    t = t_sequence[0,i]
    q_R = calculate_from_R_to_quaternion(t)
    q_sequence = np.array([[qw_sequence[0,i]],[qx_sequence[0,i]],
                          [qy_sequence[0,i]],[qz_sequence[0,i]]])
    
    # 要注意无人机路径姿态在前，姿态变换矩阵在后
    q_new = quaternion_multiply(q_sequence, q_R)

    # 经过检查无需再次归一化，但是需要确保qw>=0
    if q_new[0,0] < 0:
        q_new = -1*q_new
    
    q_answer_sequence[:,i] = q_new[:,0]

# 将计算后的数据保存在csv文件中

df_new = pd.DataFrame()
df_new['t'] = t_sequence.flatten()
df_new['qw'] = q_answer_sequence[0,:].flatten()
df_new['qx'] = q_answer_sequence[1,:].flatten()
df_new['qy'] = q_answer_sequence[2,:].flatten()
df_new['qz'] = q_answer_sequence[3,:].flatten()
df_new.to_csv("documents/tracking_end_effector.csv", index=False)


# 绘制四元数的变化曲线
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

plt.figure()
plt.plot(t_sequence.flatten(), q_answer_sequence[0,:], label="qw")
plt.plot(t_sequence.flatten(), q_answer_sequence[1,:], label="qx")
plt.plot(t_sequence.flatten(), q_answer_sequence[2,:], label="qy")
plt.plot(t_sequence.flatten(), q_answer_sequence[3,:], label="qz")

plt.xlabel("时间（秒）")
plt.ylabel("四元数")

# 设置标题
plt.title('四元数随时间变化曲线')

# 添加图例
plt.legend(loc='best', fontsize=12)

plt.grid(True, alpha=0.3)
plt.savefig("documents/question1.png")
plt.show()
