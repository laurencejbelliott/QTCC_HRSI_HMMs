#!/usr/bin/env python
# coding: utf-8

# In[38]:


import rosbag, os, matplotlib
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib.qsrlib import QSR_QTC_BC_Simplified
import numpy as np
import pandas as pd
import datetime as dt

os.chdir("/home/loz/QTC_Trajectory_HMMs/from_bags/")


# In[2]:


lab_bags = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser("~/QTC_Trajectory_HMMs/from_bags/HRSI_situation_QTC_rosbags")) for f in fn]


# In[60]:


r_positions = []
h_positions = []

r_state_seqs = []
h_state_seqs = []

qsrlib = QSRlib()

for bag_path in lab_bags:
    bag = rosbag.Bag(bag_path)
    
    r_xs = []
    r_ys = []
    
    r_state_seq = []
    for topic, msg, t in bag.read_messages(topics=['/robot5/control/odom']):
        t = t.to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        r_xs.append(x)
        r_ys.append(y)
        
        r_state_seq.append(Object_State(name="robot", timestamp=t, x=x, y=y))
    r_state_seqs.append(r_state_seq)
    
    h_xs = []
    h_ys = []
    
    h_state_seq = []
    for topic, msg, t in bag.read_messages(topics=['/robot5/people_tracker_filtered/positions']):
        t = t.to_sec()
        try:
            x = msg.poses[0].position.x
            y = msg.poses[0].position.y
            
            h_xs.append(x)
            h_ys.append(y)
            
            h_state_seq.append(Object_State(name="human", timestamp=t, x=x, y=y))
    
        except:
            pass
    h_state_seqs.append(h_state_seq)
    
    bag.close()
    r_positions.append([r_xs, r_ys])
    h_positions.append([h_xs, h_ys])


# In[61]:


# Test getting QTC_C sequence
bag_no = 0
quantisation_factor = 0.05

world = World_Trace()

h_x = [h_state_seqs[bag_no][i].x for i in range(len(h_state_seqs[bag_no]))]
h_y = [h_state_seqs[bag_no][i].y for i in range(len(h_state_seqs[bag_no]))]


r_x = [r_state_seqs[bag_no][i].x for i in range(len(r_state_seqs[bag_no]))]
r_y = [r_state_seqs[bag_no][i].y for i in range(len(r_state_seqs[bag_no]))]


# In[62]:


# Downsample state series' to 200kHz frequency
h_state_series = pd.DataFrame({"x": h_x, "y": h_y},
    index=[pd.to_datetime(h_state_seqs[bag_no][i].timestamp, unit="s") for i in range(len(h_state_seqs[bag_no]))])

h_state_series = h_state_series.resample("200ms").mean()
h_state_series


# In[63]:


r_state_series = pd.DataFrame({"x": r_x, "y": r_y},
    index=[pd.to_datetime(r_state_seqs[bag_no][i].timestamp, unit="s") for i in range(len(r_state_seqs[bag_no]))])

r_state_series = r_state_series.resample("200ms").mean()
r_state_series


# In[64]:


# Create world_trace state series from downsampled human position data
h_state_seq = []
for index, row in h_state_series.iterrows():
    x = row['x']
    y = row['y']
    t = (pd.to_datetime(index) - dt.datetime(1970,1,1)).total_seconds()
    
    h_state_seq.append(Object_State(name="human", timestamp=t, x=x, y=y))


# In[65]:


# Create world_trace state series from downsampled robot position data
r_state_seq = []
for index, row in r_state_series.iterrows():
    x = row['x']
    y = row['y']
    t = (pd.to_datetime(index) - dt.datetime(1970,1,1)).total_seconds()
    
    r_state_seq.append(Object_State(name="robot", timestamp=t, x=x, y=y))


# In[83]:


world.add_object_state_series(h_state_seq)
world.add_object_state_series(r_state_seq)

# make a QSRlib request message
dynamic_args = {"qtccs": {"no_collapse": True, "quantisation_factor": quantisation_factor,
                                        "validate": False, "qsrs_for": [("human", "robot")]}}

qsrlib_request_message = QSRlib_Request_Message(
    'qtccs', world, dynamic_args)

# request your QSRs
qsrlib_response_message = qsrlib.request_qsrs(req_msg=qsrlib_request_message)
qsrlib_response_message


# In[9]:


bag_path


# In[11]:


for bag_no in range(len(lab_bags)):

    matplotlib.rcParams['figure.figsize'] = (20.0, 10.0)
    matplotlib.rcParams['font.size'] = 30

    r_x = r_positions[bag_no][0]
    r_y = r_positions[bag_no][1]

    h_x = h_positions[bag_no][0]
    h_y = h_positions[bag_no][1]
    
    
#     r_interp = interp1d(r_x, r_y)
#     r_x = np.linspace(r_x[0], r_x[-1], len(r_x)*100)
#     r_y = r_interp(r_x)

#     h_interp = interp1d(h_x, h_y)
#     h_x = np.linspace(h_x[0], h_x[-1], len(h_x)*100)
#     h_y = h_interp(h_x)
    

    r_colour = np.array([[shade, 0, 0] for shade in np.linspace(50, 255, len(r_x))]) / 255.0
    h_colour = np.array([[0, 0, shade] for shade in np.linspace(50, 255, len(h_x))]) / 255.0

    r_size = np.linspace(10, 100, len(r_x))
    h_size = np.linspace(10, 100, len(h_x))

    plt.scatter(r_x, r_y, c=r_colour, s=r_size)
    
    # Get QTC_C sequence
    bag_path = lab_bags[bag_no]
    bag = rosbag.Bag(bag_path)
    qtc_c_seq = []
    for topic, msg, t in bag.read_messages(topics=['/robot5/qtc_state_topics']):
        qtc_c_seq.append(msg.data)
    
    qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0,0,0,0']
    
    
    plt.title(lab_bags[bag_no][67:])

    plt.scatter(h_x, h_y, c=h_colour, s=h_size)
    
    plt.savefig("lab_bag_plots/"+str(lab_bags[bag_no]).replace("/","_")[:-4]+".png")
    plt.close()
    
    print(lab_bags[bag_no]+":\n"+str(qtc_c_seq)+"\n")


# # In[151]:
#
#
# # Get path-crossing QTC_C sequences
#
# pc_seqs = []
# for bag_path in pc_bags:
#     bag_path = "sim_bags/" + bag_path
#     bag = rosbag.Bag(bag_path)
#
#     qtc_c_seq = []
#     for topic, msg, t in bag.read_messages(topics=['/robot4/qtc_state_topics']):
#         qtc_c_seq.append(str(msg.data).replace(',',''))
#
#     qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0000']
#     pc_seqs.append(qtc_c_seq)
#
#
#
# # In[152]:
#
#
# pc_seqs
#
#
# # In[153]:
#
#
# # Get overtaking QTC_C sequences
#
# ot_seqs = []
# for bag_path in ot_bags:
#     bag_path = "sim_bags/" + bag_path
#     bag = rosbag.Bag(bag_path)
#
#     qtc_c_seq = []
#     for topic, msg, t in bag.read_messages(topics=['/robot4/qtc_state_topics']):
#         qtc_c_seq.append(str(msg.data).replace(',',''))
#
#     qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0000']
#     ot_seqs.append(qtc_c_seq)
#
#
#
# # In[154]:
#
#
# ot_seqs
#
#
# # In[158]:
#
#
# # Get pass-by QTC_C sequences
#
# pb_seqs = []
# for bag_path in pb_bags:
#     bag_path = "sim_bags/" + bag_path
#     bag = rosbag.Bag(bag_path)
#
#     qtc_c_seq = []
#     for topic, msg, t in bag.read_messages(topics=['/robot4/qtc_state_topics']):
#         qtc_c_seq.append(str(msg.data).replace(',',''))
#
#     qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0000']
#     pb_seqs.append(qtc_c_seq)
#
#
#
# # In[159]:
#
#
# pb_seqs
#
#
# # In[167]:
#
#
# np.save("pc_sim_seqs", pc_seqs)
# np.save("ot_sim_seqs", ot_seqs)
# np.save("pb_sim_seqs", pb_seqs)
#
#
# # In[ ]:
#
#
# np.load("pb_sim_seqs.npy", allow_pickle=True)
#
#
# # In[ ]:
#
#
#
#
