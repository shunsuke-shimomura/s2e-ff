import pathlib  
from common import read_3d_vector_from_csv, read_scalar_from_csv
import numpy as np
import matplotlib.pyplot as plt

dirs = pathlib.Path("../../data/logs").glob("logs*")
cutoff = 601

m_fig = plt.figure()
m_ax = m_fig.add_subplot(111)
m_ax.set_title("worst value of position estimate error")
m_ax.set_xlabel("time [s]")
m_ax.set_ylabel("worst value of position estimate error [m]")

v_fig = plt.figure()
v_ax = v_fig.add_subplot(111)
v_ax.set_title("std of position estimate error")
v_ax.set_xlabel("time [s]")
v_ax.set_ylabel("std of position estimate error [m]")

e_fig = plt.figure()
e_ax = e_fig.add_subplot(111)
e_ax.set_title("Monte Calro")
e_ax.set_xlabel("time [s]")
e_ax.set_ylabel("estimated pos [m]")

n_ = 0.0
mean = 0.0
M2 = 0.0
n = 0
mean_p = 0.0
M2_p = 0.0
for i,d in enumerate(dirs):
    if i > 30:
        break
    print("{0} \r".format(i))
    file_name = list(d.glob("*.csv"))[0]
    t = read_scalar_from_csv(file_name, "elapsed_time[s]")
    pos = read_3d_vector_from_csv(file_name, 'satellite1_position_from_satellite0_rtn', 'm')
    estimated_pos = np.array([
        read_scalar_from_csv(file_name, 'kalman_filter_estimated_state_rtn(0)[m]')[0],
        read_scalar_from_csv(file_name, 'kalman_filter_estimated_state_rtn(1)[m]')[0],
        read_scalar_from_csv(file_name, 'kalman_filter_estimated_state_rtn(2)[m]')[0]
    ],
    )
    error_x = pos[0][cutoff:]-estimated_pos[0][cutoff:]
    error_y = pos[1][cutoff:]-estimated_pos[1][cutoff:]
    error_z = pos[2][cutoff:]-estimated_pos[2][cutoff:]
    error_p = np.sqrt(error_x ** 2 + error_y ** 2 + error_z ** 2)

    n_ += 1
    delta = error_x - mean
    mean += delta / n_
    delta2 = error_x - mean
    M2 += delta * delta2

    variance = M2 / (n_ - 1)  

    alotn = 0
    alotmean = 0.0
    alotM2 = 0.0
    alotworse = np.zeros(30)
    for i in range(len(error_p)//30):
        n += 1
        alotn += 1
        ep_part = error_p[i*30:(i+1)*30]
        
        alotdelta = ep_part - alotmean
        alotmean += alotdelta / alotn
        alotdelta2 = ep_part - alotmean
        alotM2 += alotdelta * alotdelta2

        delta_p = ep_part - mean_p
        mean_p += delta_p / n
        delta2_p = ep_part - mean_p
        M2_p += delta_p * delta2_p

        for j in range(len(ep_part)):
            if ep_part[j] > alotworse[j]:
                alotworse[j] = ep_part[j]

    if n < 2:
        variance_alot = float('nan')  # Not enough data to calculate variance
    else:
        variance_alot = alotM2 / (alotn - 1)

        m_ax.plot(np.arange(0,3,0.1),alotworse,color='black',alpha=0.2)
        v_ax.plot(np.arange(0,3,0.1),np.sqrt(variance_alot),color='black',alpha=0.2)
        e_ax.plot(t[0][cutoff:],error_x,color="#1f77b4",alpha=0.4)

    if n < 2:
        variance_p = float('nan')  # Not enough data to calculate variance
    else:
        variance_p = M2_p / (n - 1)


fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111)
ax.set_title("mean of position estimate error")
ax.set_xlabel("time [s]")
ax.set_ylabel("mean of position estimate error [m]")

ax.plot(np.arange(0,3,0.1),mean_p)

fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111)
ax.set_title("std of position estimate error")
ax.set_xlabel("time [s]")
ax.set_ylabel("std of position estimate error [m]")

ax.plot(np.arange(0,3,0.1),np.sqrt(variance_p))

ax.legend()

ax.plot(np.arange(0,3,0.1),mean_p)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title("std of position estimate error")
ax.set_xlabel("time [s]")
ax.set_ylabel("std of position estimate error [m]")

ax.plot(t[0][cutoff:],np.sqrt(variance))

ax.legend()

h_inf = np.array(
    
    [(0.0, 0.4622796841136908), (0.1, 0.47869139405924116), (0.2, 0.49706588484688874), (0.30000000000000004, 0.5226729784047286), (0.4, 0.5491022397588531), (0.5, 0.5762270418635498), (0.6000000000000001, 0.6156214943401382), (0.7000000000000001, 0.6459939432805856), (0.8, 0.6771652852275765), (0.9, 0.7228410904211809), (1.0, 0.46069852421668706), (1.1, 0.47645078703854843), (1.2000000000000002, 0.4995638467559245), (1.3, 0.5220910001472192), (1.4000000000000001, 0.5468633216798312), (1.5, 0.5825933430634046), (1.6, 0.6118229986216165), (1.7000000000000002, 0.6417811252544291), (1.8, 0.685854410727871), (1.9000000000000001, 0.7185755575522805), (2.0, 0.4583001203818604), (2.1, 0.47999686473152703), (2.2, 0.4988984149488296), (2.3000000000000003, 0.5204183879097476), (2.4000000000000004, 0.5512336215516738), (2.5, 0.5793305756274922), (2.6, 0.6080051804905038), (2.7, 0.6500649529401226), (2.8000000000000003, 0.6816054984346323), (2.9000000000000004, 0.7139678196328245)]

)

nominal = np.array([(0.0, 0.0802536539338513), (0.1, 0.08510039166354366), (0.2, 0.08741580682342408), (0.30000000000000004, 0.09262650152880142), (0.4, 0.09842243440276127), (0.5, 0.10006851000368942), (0.6000000000000001, 0.1058664688692679), (0.7000000000000001, 0.11217396329516746), (0.8, 0.11299890955465865), (0.9, 0.07657722229950167), (1.0, 0.08098500743631254), (1.1, 0.08342742675866471), (1.2000000000000002, 0.0883476520299172), (1.3, 0.09388418267447579), (1.4000000000000001, 0.09578541487537999), (1.5, 0.10142940420650387), (1.6, 0.10760505922116113), (1.7000000000000002, 0.10870403513113006), (1.8, 0.11471248926342897), (1.9000000000000001, 0.07713276135368795), (2.0, 0.07962566652715135), (2.1, 0.08419728031440366), (2.2, 0.08941828397169965), (2.3000000000000003, 0.09154798854330504), (2.4000000000000004, 0.09699749086927764), (2.5, 0.1030039464866692), (2.6, 0.10437975240778267), (2.7, 0.11029724395785313), (2.8000000000000003, 0.11670499336872615), (2.9000000000000004, 0.07608941373181229)]

)
print(h_inf.shape,nominal.shape)
m_ax.plot(h_inf[:,0],[*h_inf[1:,1],h_inf[0,1]],color="r")
v_ax.plot(nominal[:,0],nominal[:,1],color="r")
plt.show()