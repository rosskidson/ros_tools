import rosbag

with rosbag.Bag('output3.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('kitty_05.bag').read_messages():
        if "image_rect" in topic:
            msg.step = 1226
            outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, t)
