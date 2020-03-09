#!/usr/bin/env python
# coding: utf-8

# In[2]:


import rosbag, os, matplotlib
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
import numpy as np


# In[3]:


os.listdir("sim_bags")


# In[4]:


sim_bags = os.listdir("sim_bags")
sim_bags

no_collide_bag_codes = [
    "S1-i25-e35",
    "S1-i25-e59",
    "S2-i25-e35",
    "S2-i25-e59",
    "S3-i33-e40",
    "S3-i36-e15",
    "S4-i33-e40",
    "S4-i36-e15",
    "S6-i33-e40",
    "S6-i36-e15",
    "S7-i33-e40",
    "S7-i36-e15"
]

sim_bags = [bag for bag in sim_bags if bag[:10] in no_collide_bag_codes]
sim_bags


# In[5]:


sim_r_positions = []
sim_h_positions = []

for bag_path in sim_bags:
    bag_path = "sim_bags/"+bag_path
    bag = rosbag.Bag(bag_path)
    
    r_xs = []
    r_ys = []
    for topic, msg, t in bag.read_messages(topics=['/robot4/robot_pose']):
        r_xs.append(msg.pose.position.x)
        r_ys.append(msg.pose.position.y)    
    
    h_xs = []
    h_ys = []
    for topic, msg, t in bag.read_messages(topics=['/robot4/human_perception/tracked_persons']):
        try:
            h_xs.append(msg.tracks[0].pose.pose.position.x)
            h_ys.append(msg.tracks[0].pose.pose.position.y)
        except:
            print("No human detection\n")
    
    bag.close()
    sim_r_positions.append([r_xs, r_ys])
    sim_h_positions.append([h_xs, h_ys])


# In[6]:


for bag_no in range(len(sim_bags)):

    matplotlib.rcParams['figure.figsize'] = (20.0, 10.0)
    matplotlib.rcParams['font.size'] = 30

    r_x = sim_r_positions[bag_no][0]
    r_y = sim_r_positions[bag_no][1]

    h_x = sim_h_positions[bag_no][0]
    h_y = sim_h_positions[bag_no][1]

    r_colour = np.array([[shade, 0, 0] for shade in np.linspace(50, 255, len(r_x))]) / 255.0
    h_colour = np.array([[0, 0, shade] for shade in np.linspace(50, 255, len(h_x))]) / 255.0

    r_size = np.linspace(10, 100, len(r_x))
    h_size = np.linspace(10, 100, len(h_x))

    plt.scatter(r_x, r_y, c=r_colour, s=r_size)
    
    # Get QTC_C sequence
    bag_path = "sim_bags/"+sim_bags[bag_no]
    bag = rosbag.Bag(bag_path)
    qtc_c_seq = []
    for topic, msg, t in bag.read_messages(topics=['/robot4/qtc_state_topics']):
        qtc_c_seq.append(msg.data)
    
    qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0,0,0,0']
    
    
    plt.title(sim_bags[bag_no])

    plt.scatter(h_x, h_y, c=h_colour, s=h_size)
    
    plt.savefig("sim_bag_plots/"+sim_bags[bag_no][:-4]+".png")
    plt.close()
    
    print(sim_bags[bag_no]+":\n"+str(qtc_c_seq)+"\n")


# In[7]:


failed_bags = []
for bag_no in range(len(sim_bags)):

    matplotlib.rcParams['figure.figsize'] = (20.0, 10.0)
    matplotlib.rcParams['font.size'] = 30

    r_x = sim_r_positions[bag_no][0]
    r_y = sim_r_positions[bag_no][1]

    h_x = sim_h_positions[bag_no][0]
    h_y = sim_h_positions[bag_no][1]

    r_colour = np.array([[shade, 0, 0] for shade in np.linspace(50, 255, len(r_x))]) / 255.0
    h_colour = np.array([[0, 0, shade] for shade in np.linspace(50, 255, len(h_x))]) / 255.0

    r_size = np.linspace(10, 100, len(r_x))
    h_size = np.linspace(10, 100, len(h_x))

    plt.scatter(r_x, r_y, c=r_colour, s=r_size)
    
    
    plt.title(sim_bags[bag_no])

    plt.scatter(h_x, h_y, c=h_colour, s=h_size)
    
    plt.show()
    
    bag_success = input("Was the interaction simulated sucessfully? N for no:\n")
    print("\n")
    if bag_success.upper() == "N":
        failed_bags.append(sim_bags[bag_no])
        print("The above simulation failed.\n")
    else:
        print("The above simulation passed.\n")
    
    plt.close()


# In[ ]:


failed_bags


# In[9]:


valid_sim_bags = [bag for bag in sim_bags if bag not in failed_bags]
valid_sim_bags


# In[ ]:


len(valid_sim_bags)


# In[8]:


pc_bag_codes = [
    "S1-i25-e35",
    "S1-i25-e59",
    "S2-i25-e35",
    "S2-i25-e59"
]

ot_bag_codes = [
    "S3-i33-e40",
    "S3-i36-e15",
    "S4-i33-e40",
    "S4-i36-e15"
]

pb_bag_codes = [
    "S6-i33-e40",
    "S6-i36-e15",
    "S7-i33-e40",
    "S7-i36-e15"
]

pc_bags = [bag for bag in valid_sim_bags if bag[:10] in pc_bag_codes]
ot_bags = [bag for bag in valid_sim_bags if bag[:10] in ot_bag_codes]
pb_bags = [bag for bag in valid_sim_bags if bag[:10] in pb_bag_codes]


# In[ ]:


# Get path-crossing QTC_C sequences

pc_seqs = []
for bag_path in pc_bags:
    bag_path = "sim_bags/" + bag_path
    bag = rosbag.Bag(bag_path)
    
    qtc_c_seq = []
    for topic, msg, t in bag.read_messages(topics=['/robot4/qtc_state_topics']):
        qtc_c_seq.append(str(msg.data).replace(',',''))
    
    qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0000']
    pc_seqs.append(qtc_c_seq)
    


# In[ ]:


pc_seqs


# In[ ]:


# Get overtaking QTC_C sequences

ot_seqs = []
for bag_path in ot_bags:
    bag_path = "sim_bags/" + bag_path
    bag = rosbag.Bag(bag_path)
    
    qtc_c_seq = []
    for topic, msg, t in bag.read_messages(topics=['/robot4/qtc_state_topics']):
        qtc_c_seq.append(str(msg.data).replace(',',''))
    
    qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0000']
    ot_seqs.append(qtc_c_seq)
    


# In[ ]:


ot_seqs


# In[1]:


# Get pass-by QTC_C sequences

pb_seqs = []
for bag_path in pb_bags:
    bag_path = "sim_bags/" + bag_path
    bag = rosbag.Bag(bag_path)
    
    qtc_c_seq = []
    for topic, msg, t in bag.read_messages(topics=['/robot4/qtc_state_topics']):
        qtc_c_seq.append(str(msg.data).replace(',',''))
    
    qtc_c_seq = [qtc for qtc in qtc_c_seq if qtc != '0000']
    pb_seqs.append(qtc_c_seq)
    
    
pb_seqs = [seq for seq in pb_seqs]


# In[159]:


pb_seqs


# In[167]:


np.save("pc_sim_seqs", pc_seqs)
np.save("ot_sim_seqs", ot_seqs)
np.save("pb_sim_seqs", pb_seqs)


# In[ ]:


np.load("pb_sim_seqs.npy", allow_pickle=True)


# In[ ]:




