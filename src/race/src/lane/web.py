#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import String
from race.msg import drive_param
import logging
from websocket_server import WebsocketServer
import threading

rospy.init_node('web_socket_server')
path_list = []
new_path = 1
new_path_test = "]"
new_drive = 1
new_car_param = 1
drive_test = "d"
dest_test = 'p'
counter_test = 1
driver_vel = 0
driver_ang = 0
car_pos = 0
car_orient = 0
destination = []
received_destination = 0

class Update (threading.Thread):
    def __init__(self):
        print 'Update started'
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()

    def run(self):
        global server, path_pub, path_list, new_path, new_drive, new_path_test, new_car_param, counter_test
        global driver_test, driver_vel, driver_ang, car_pos, car_orient
        while True:
            # test code:
            # if counter_test == 0:
            #     print 'new guy, publish message'
            #     new_path = 1
            # else:
            #     new_path = 0

            if new_path == 1:
                print 'publishing...'
                # test code:
                # path_list = new_path_test+"[[2.948,3.616],[2.980396064627311,3.639272550225596],[3.010616637020841,3.661183636216565],[3.038790032154178,3.681773792481133],[3.0650445650009153,3.701083553527529],[3.0895085505346422,3.7191534538639828],[3.112310303728949,3.736024027998723],[3.1335781395574274,3.751735810439977],[3.153440372993667,3.766329335695975],[3.1720253190112593,3.7798451382749434],[3.189461292583795,3.7923237526851135],[3.205876608684864,3.8038057134347123],[3.221399582288057,3.814331555031968],[3.2361585283669645,3.82394181198511],[3.250281761895178,3.832677018802366],[3.2638975978462876,3.840577709991966],[3.2771343511938835,3.8476844200621376],[3.2901203369115573,3.8540376835211103],[3.3029838699728993,3.8596780348771125],[3.315853265351499,3.8646460086383714],[3.32885683802095,3.868982139313117],[3.342122902954839,3.8727269614095774],[3.3557797751267597,3.8759210094359817],[3.3699557695103,3.8786048179005572],[3.384779201079054,3.8808189213115343],[3.4003779876156464,3.88260360605675],[3.416831986796171,3.883969135956725],[3.434127716420392,3.884867466540087],[3.452240970132068,3.885243854084902],[3.471147541574962,3.885043554869241],[3.4908232243928325,3.8842118251711697],[3.511243812229442,3.8826939212687583],[3.5323850987285486,3.880435099440074],[3.554222877533914,3.8773806159631867],[3.576732942289298,3.873475727116162],[3.5998910866384612,3.8686656891770697],[3.623673104225164,3.8628957584239783],[3.6480547886931682,3.8561111911349557],[3.673011933686232,3.84825724358807],[3.698520332848117,3.83927917206139],[3.724555779822584,3.829122232832984],[3.751094068253393,3.81773168218092],[3.7781109917843034,3.805052776383265],[3.8055823440590766,3.7910307717180896],[3.8334839187214738,3.77561092446346],[3.8617915094152546,3.758738490897447],[3.8904809097841784,3.7403587272981156],[3.9195279134720074,3.7204168899435364],[3.948908314122501,3.698858235111777],[3.9785979053794205,3.675628019080906],[4.008571429199451,3.6506743765196967],[4.03877943873656,3.6240116450831654],[4.069148298342001,3.5957203654125665],[4.0996033206799485,3.565883956539862],[4.130069818414581,3.5345858374970125],[4.160473104210076,3.5019094273159803],[4.190738490730607,3.4679381450287265],[4.220791290640352,3.4327554096672124],[4.250556816603489,3.3964446402633985],[4.2799603812841935,3.359089255849248],[4.308927297346643,3.3207726754567224],[4.337382877455013,3.2815783181177807],[4.365252434273482,3.241589602864387],[4.392461280466225,3.2008899487285007],[4.418934728697419,3.159562774742086],[4.44459809163124,3.117691499937101],[4.469376681931867,3.0753595433455083],[4.493195812263475,3.0326503239992704],[4.5159807952902415,2.9896472609303477],[4.537656943676343,2.9464337731707024],[4.5581495700859564,2.903093279752295],[4.577383987183256,2.8597091997070874],[4.595285507632421,2.8163649520670417],[4.611779444097627,2.7731439558641173],[4.626791109243053,2.730129630130278],[4.640237755242901,2.687392703934395],[4.651966478154946,2.644893456666614],[4.661788251100422,2.602535297882498],[4.6695137486638965,2.5602211671389785],[4.674953645429933,2.5178540039929866],[4.677918615983102,2.4753367480014523],[4.678219334907966,2.4325723387213087],[4.675666476789094,2.3894637157094856],[4.670070716211054,2.3459138185229143],[4.66124272775841,2.3018255867185258],[4.648993186015729,2.2571019598532507],[4.633132765567578,2.2116458774840213],[4.613472140998525,2.1653602791677677],[4.589821986893134,2.118148104461422],[4.561992977835973,2.0699122929219134],[4.529795788411609,2.0205557841061754],[4.493041093204608,1.969981517571138],[4.4515395667995366,1.9180924328737312],[4.4051018837809615,1.8647914695708878],[4.35353871873345,1.809981567219538],[4.2966607462415665,1.7535656653766134],[4.234278640889881,1.6954467035990453],[4.166203077262957,1.6355276214437633],[4.092244729945364,1.5737113584677003],[4.012214273521666,1.5099008542277863]]"
                # counter_test = 1
                
                server.send_message_to_all(new_path_test+str(path_list))
                new_path = 0
            if type(car_pos) == list and type(car_orient) == list:
                if car_pos == [0,0] and car_orient == [1,0]:
                    return
                if new_drive == 1 or new_car_param == 1:
                    msg_pub = drive_test+str(driver_vel)+","+str(driver_ang)+","+str(car_pos[0])+","+str(car_pos[1])+","+str(car_orient[0])+","+str(car_orient[1])
                    print 'publish drive_param: '+msg_pub
                    server.send_message_to_all(msg_pub)
                    new_drive = 0

def lin_map(val, in_left, in_right, out_left, out_right):
    k = (float(out_right) - out_left)/(in_right - in_left)
    b = out_left - k*in_left
    return k*val+b

def map_velocity(v):
    # unit: m/s
    v_map = 0
    if v > 0:
            if v >= 15.5 and v <= 20:
                    v_map = lin_map(float(v), 15.5, 20.0, 0.0, 0.304)
                    # v_map = arduino_map(float(v), 15.5, 20, 0, 0.304)
            elif v > 20:
                    v_map = 0.304
    elif v < 0:
            if v <= -15.5 and v >= -20:
                    v_map = lin_map(float(v), -20, -15.5, -0.289, 0)
            elif v < -20:
                    v_map = -0.289
    else:
            v_map = 0
    return v_map

def map_angle(angle):
        # unit: radians
        # In this case: counter-clockwise negative, clockwise positive
        ang_map = lin_map(float(angle), -100, 100, -0.431139, 0.431139)
        ang_map = np.degrees(ang_map)
        return ang_map

def new_client(client, server):
    global counter_test
    server.send_message_to_all("Hey all, a new client has joined us")
    counter_test = 0
    new_drive = 1

def new_message(client, server, message):
    global dest_test, path_pub, new_path_test, destination, received_destination, path_pub
    m = message.decode('unicode_escape').encode("utf-8")
    print 'Received message: ', repr(m), ' from ', client['address']
    if type(m) == str:
        print 'first byte: ', m[0]
        try:
            if m[0] == dest_test:
                data = m[1:].split(',')
                dest = [eval(data[0]),eval(data[1])]
                print 'dest: ', dest
                if dest == destination:
                    print 'destination not changed.'
                    return
                destination = dest
                received_destination = 1
                # path_pub.publish(str(destination))
                received_destination = 0
                print 'Received destination: ', destination
        except:
            return

def client_left(client, server):
    server.send_message_to_all("Bye bye mother-fucker")

def callback_path(string):
    global path_list, new_path
    path_list = eval(string.data)
    print 'received path: ', len(path_list)
    new_path = 1

def callback_drive(data):
    global driver_vel, driver_ang, new_drive
    if abs(driver_vel - data.velocity) < 0.001 and abs(driver_ang - data.angle) < 0.001:
        print 'drive parameter does not change'
        new_drive = 0
        return
    driver_vel = data.velocity
    driver_ang = data.angle
    driver_vel = map_velocity(driver_vel)
    driver_ang = map_angle(driver_ang)
    # driver_ang = -1*driver_ang #counter-clockwise is positive
    new_drive = 1

def callback_car(data):
    global car_pos, car_orient, new_car_param
    string = data.data
    string_list = string.split(";")
    idx0 = string_list[0].find("[")
    car_pos_tmp = eval(string_list[0][idx0:])
    pos_check = 1
    if np.linalg.norm(np.array(car_pos_tmp) - np.array(car_pos)) < 0.05:
        # print 'car position did not change'
        pos_check = 0
    idx0 = string_list[1].find("[")
    car_orient_tmp = eval(string_list[1][idx0:])
    orient_check = 1
    if np.linalg.norm(np.array(car_orient_tmp) - np.array(car_orient)) < 0.05:
        # print 'car orientation did not change'
        pos_check = 0
    if car_pos_tmp == [0,0] and car_orient_tmp == [1,0]:
        return
    car_pos = car_pos_tmp
    car_orient = car_orient_tmp
    new_car_param = pos_check + orient_check
    if new_car_param < 2:
        new_car_param = 0
    else:
        new_car_param = 1

server = WebsocketServer(9090, host='172.27.165.101')
server.set_fn_new_client(new_client)
server.set_fn_message_received(new_message)
server.set_fn_client_left(client_left)

path_pub = rospy.Publisher('web_socket/path', String, queue_size=1)
rospy.Subscriber('lane_driver/path', String, callback_path)
rospy.Subscriber("lane_driver/car", String, callback_car)
rospy.Subscriber("drive_param_fin", drive_param, callback_drive)
update_thread = Update()

server.run_forever()