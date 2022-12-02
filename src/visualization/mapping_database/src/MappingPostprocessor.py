import rospy
import math
import cv2
import numpy as np
import sqlite3
import argparse
import time

class MappingPostprocessor:
    LANE = 1
    INITIAL_GPS_LON = 0#19.8103713989
    INITIAL_GPS_LAT = 0#50.0721092224
    INITIAL_GPS_HEADING = 0#6.13200807571 #radians

    def __init__(self, Size=500):
        self.Size = Size
        self.azimuth = 0
        self.number_of_nearest_points = 200
        self.local_index = 0
        self.main_index = 0
        self.tail_index = 0
        self.find_next_point_calls_counter = 0

        self.angle_range_limit = 10 * np.pi/180.0
        self.distance_limit = 25

        self.angle_range_limit_sidelook = 140 * np.pi/180.0
        self.distance_limit_sidelook = 9
        self.floor_limit_sidelook = 1

        #test
        self.second_processing_flag = False
        #tester

        self.line_0_id = 1
        self.line_1_id = 2
        self.line_2_id = 3
        self.line_3_id = 5
        self.line_rest_id = 10
        self.line_0_id_list = []
        self.line_1_id_list = []
        self.line_2_id_list = []
        self.line_3_id_list = []
        self.line_0_az_list = []
        self.line_1_az_list = []
        self.line_2_az_list = []
        self.line_3_az_list = []
        self.temp_line_0_id_list = []
        self.temp_line_1_id_list = []
        self.temp_line_2_id_list = []
        self.temp_line_3_id_list = []
        self.forbidden_index_list = []
        self.dupa = []
        self.stop_iterating = False
        self.next_iteration_over_available_points_flag = True
        self.superficial_lane = MappingPostprocessor.LANE

    def get_dataset(self):
        self.connector_in = sqlite3.connect('mapping_database.sqlite')
        self.cursor_in = self.connector_in.cursor()
        self.cursor_in.execute('''SELECT id, lineId , lon, lat, alt FROM LinePoints''')
        self.dataset_in = self.cursor_in.fetchall()
        self.dataset_out = self.dataset_in
        self.connector_out = sqlite3.connect('mapping_database_MP.sqlite')
        self.cursor_out = self.connector_out.cursor()
        self.cursor_out.execute('''DROP TABLE IF EXISTS 'LinePoints' ''')
        self.cursor_out.execute('''
                            CREATE TABLE 'LinePoints' ( 'id' INTEGER PRIMARY KEY NOT NULL , 'lineId' INTEGER , 
                            'lon' REAL NOT NULL , 'lat' REAL NOT NULL , 'alt' REAL NOT NULL , FOREIGN KEY(lineId) REFERENCES Line(id) )
                            ''')

    def save_dataset(self):
        for i in range(len(self.dataset_out)):
            self.cursor_out.execute('''INSERT INTO LinePoints(id, lineId , lon, lat, alt)
                                VALUES(?,?,?,?,?)''', (self.dataset_out[i][0], self.dataset_out[i][1], 
                                self.dataset_out[i][2], self.dataset_out[i][3], self.dataset_out[i][4]))
        self.connector_out.commit()

    def clear_ids(self):
        for i in range(len(self.dataset_out)):
            self.dataset_out[i] = list(self.dataset_out[i])
            self.dataset_out[i][1] = self.line_rest_id
            self.dataset_out[i] = tuple(self.dataset_out[i])

    def refract_ids(self):
        n = []
        for i in range(len(self.dataset_out)):
            self.dataset_out[i] = list(self.dataset_out[i])
            
            if (i in self.line_0_id_list):
                self.dataset_out[i][1] = self.line_0_id
            elif (i in self.line_1_id_list):
                self.dataset_out[i][1] = self.line_1_id
            elif (i in self.line_2_id_list):
                self.dataset_out[i][1] = self.line_2_id
            elif (i in self.line_3_id_list):
                self.dataset_out[i][1] = self.line_3_id
            else: 
                self.dataset_out[i][1] = 0 #self.line_rest_id
        

            self.dataset_out[i] = tuple(self.dataset_out[i])


    def get_distance(self, tail_lon, tail_lat, head_lon, head_lat):
        tail_lon = tail_lon * np.pi/180.0
        tail_lat = tail_lat * np.pi/180.0
        head_lon = head_lon * np.pi/180.0
        head_lat = head_lat * np.pi/180.0 
        a = math.sin((head_lat - tail_lat)/2.0)**2 + math.cos(tail_lat) * math.cos(head_lat) * math.sin((head_lon - tail_lon)/2.0)**2 
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        R = 6371000
        return R*c
    
    def get_azimuth(self, tail_lon, tail_lat, head_lon, head_lat):
        l1 = tail_lon * np.pi/180.0
        l2 = head_lon * np.pi/180.0
        fi1 = tail_lat * np.pi/180.0
        fi2 = head_lat * np.pi/180.0
        azimuth = math.atan2(math.sin(l2-l1)*math.cos(fi2), 
                            math.cos(fi1)*math.sin(fi2) - math.sin(fi1)*math.cos(fi2)*math.cos(l2-l1))
        if azimuth < 0: azimuth = azimuth + 2*np.pi
        return azimuth

    
    def check_point_in_wide_area(self, search_azimuth, tail_lon, tail_lat):
        search_azimuth = (search_azimuth + 2*np.pi) % (2*np.pi)
        distance = self.get_distance(tail_lon, tail_lat, self.dataset_in[self.local_index][2], self.dataset_in[self.local_index][3])
        if distance > self.distance_limit_sidelook or distance < self.floor_limit_sidelook:
            return False
        azimuth = self.get_azimuth(tail_lon, tail_lat, self.dataset_in[self.local_index][2], self.dataset_in[self.local_index][3])
        if ( search_azimuth + self.angle_range_limit_sidelook/2.0 ) < azimuth or ( search_azimuth - self.angle_range_limit_sidelook/2.0 ) > azimuth:
            return False
        else:
            return True

    def take_closest_one(self, tail_lon, tail_lat, index_list):
        dist = 30
        found_index = -1
        for i in range(len(index_list)):
            dist_temp = self.get_distance(tail_lon, tail_lat, self.dataset_in[index_list[i]][2], self.dataset_in[index_list[i]][3])
            if self.second_processing_flag == True: print [found_index, dist_temp]
            if dist_temp < dist and dist_temp > self.floor_limit_sidelook:
                dist = dist_temp
                found_index = index_list[i]
        
        return found_index


    def horizontal_initial_search_take_closest_ones(self):
        if MappingPostprocessor.LANE == 1:
            if len(self.temp_line_0_id_list) > 0 and len(self.line_0_id_list) == 0:
                self.line_0_id_list.append(self.take_closest_one(self.dataset_in[self.line_1_id_list[0]][2] , 
                                                                self.dataset_in[self.line_1_id_list[0]][3], self.temp_line_0_id_list))
            if len(self.temp_line_1_id_list) > 0 and len(self.line_1_id_list) == 0:
                self.line_1_id_list.append(self.take_closest_one(MappingPostprocessor.INITIAL_GPS_LON , 
                                                                MappingPostprocessor.INITIAL_GPS_LAT, self.temp_line_1_id_list))
            if len(self.temp_line_2_id_list) > 0 and len(self.line_2_id_list) == 0:
                self.line_2_id_list.append(self.take_closest_one(MappingPostprocessor.INITIAL_GPS_LON , 
                                                                MappingPostprocessor.INITIAL_GPS_LAT, self.temp_line_2_id_list))
            if len(self.temp_line_3_id_list) > 0 and len(self.line_3_id_list) == 0:
                self.line_3_id_list.append(self.take_closest_one(self.dataset_in[self.line_2_id_list[0]][2] , 
                                                                self.dataset_in[self.line_2_id_list[0]][3], self.temp_line_3_id_list))

    def horizontal_initial_search_closest_lines(self):
        if MappingPostprocessor.LANE == 1:
            if self.check_point_in_wide_area(MappingPostprocessor.INITIAL_GPS_HEADING + np.pi/2.0, 
                                            MappingPostprocessor.INITIAL_GPS_LON, MappingPostprocessor.INITIAL_GPS_LAT):
                self.temp_line_1_id_list.append(self.local_index)
            elif self.check_point_in_wide_area(MappingPostprocessor.INITIAL_GPS_HEADING - np.pi/2.0, 
                                            MappingPostprocessor.INITIAL_GPS_LON, MappingPostprocessor.INITIAL_GPS_LAT):
                self.temp_line_2_id_list.append(self.local_index)


    def horizontal_initial_search_further_lines(self):
        if MappingPostprocessor.LANE == 1:
            if self.check_point_in_wide_area(MappingPostprocessor.INITIAL_GPS_HEADING + np.pi/2.0, 
                                            self.dataset_in[self.line_1_id_list[0]][2] , self.dataset_in[self.line_1_id_list[0]][3]):
                self.temp_line_0_id_list.append(self.local_index)
            elif self.check_point_in_wide_area(MappingPostprocessor.INITIAL_GPS_HEADING - np.pi/2.0, 
                                            self.dataset_in[self.line_2_id_list[0]][2] , self.dataset_in[self.line_2_id_list[0]][3]):
                self.temp_line_3_id_list.append(self.local_index)

    

    ##########################################################################################################

    def init_frontal_search_base(self):
        self.line_0_az_list.append(MappingPostprocessor.INITIAL_GPS_HEADING)
        self.line_1_az_list.append(MappingPostprocessor.INITIAL_GPS_HEADING)
        self.line_2_az_list.append(MappingPostprocessor.INITIAL_GPS_HEADING)
        self.line_3_az_list.append(MappingPostprocessor.INITIAL_GPS_HEADING)
        self.temp_line_0_id_list = []
        self.temp_line_1_id_list = []
        self.temp_line_2_id_list = []
        self.temp_line_3_id_list = []

    def init_frontal_search_partial(self):
        self.temp_line_0_id_list = []
        self.temp_line_1_id_list = []
        self.temp_line_2_id_list = []
        self.temp_line_3_id_list = []
    
    def check_point_in_area(self, search_azimuth, tail_lon, tail_lat):
        search_azimuth = (search_azimuth + 2*np.pi) % (2*np.pi)
        distance = self.get_distance(tail_lon, tail_lat, self.dataset_in[self.local_index][2], self.dataset_in[self.local_index][3])

        if distance > self.distance_limit or (tail_lon == self.dataset_in[self.local_index][2] and tail_lat == self.dataset_in[self.local_index][3]):
            return False
        azimuth = self.get_azimuth(tail_lon, tail_lat, self.dataset_in[self.local_index][2], self.dataset_in[self.local_index][3])
        if ( search_azimuth + self.angle_range_limit/2.0 ) < azimuth or ( search_azimuth - self.angle_range_limit/2.0 ) > azimuth:
            return False
        else:
            return True

    def take_with_smaller_angle_declination(self, search_azimuth, tail_lon, tail_lat, index_list):
        search_azimuth = (search_azimuth + 2*np.pi) % (2*np.pi)
        dec = 4
        for i in range(len(index_list)):

            az_temp = self.get_azimuth(tail_lon, tail_lat, self.dataset_in[index_list[i]][2], self.dataset_in[index_list[i]][3])
            #######
            #print [az_temp*180/np.pi, search_azimuth*180/np.pi]
            #print '--'
            if (az_temp < np.pi and search_azimuth >= np.pi) or (az_temp >= np.pi and search_azimuth < np.pi):
                #print [az_temp*180/np.pi, search_azimuth*180/np.pi]
                if search_azimuth >= np.pi: search_azimuth = 2*np.pi - search_azimuth
                elif az_temp >= np.pi: az_temp = 2*np.pi - az_temp
                declination = az_temp + search_azimuth
            else:
                declination = math.fabs(az_temp - search_azimuth)
                #print '+++'
                #print [az_temp*180/np.pi, search_azimuth*180/np.pi]
                #print [tail_lon, tail_lat, self.dataset_in[index_list[i]][2], self.dataset_in[index_list[i]][3], len(index_list)]
                #print '+++'
            #######
            if declination < dec: 
                dec = declination
                found_index = index_list[i]
                self.dupa.append( (az_temp - search_azimuth)*180/np.pi)

        #############################################################################################################################
        #############################################################################################################################
        return found_index

    def frontal_search_next_step(self, line):
        if line == 0:
            if self.check_point_in_area(self.line_0_az_list[-1], 
                                        self.dataset_in[self.line_0_id_list[-1]][2], self.dataset_in[self.line_0_id_list[-1]][3]):
                if not (self.local_index in self.forbidden_index_list) and not (self.local_index in self.line_0_id_list):
                    self.temp_line_0_id_list.append(self.local_index)
        if line == 1:
            if self.check_point_in_area(self.line_1_az_list[-1], 
                                        self.dataset_in[self.line_1_id_list[-1]][2], self.dataset_in[self.line_1_id_list[-1]][3]):
                if not (self.local_index in self.forbidden_index_list):
                    self.temp_line_1_id_list.append(self.local_index)
        if line == 2:
            if self.check_point_in_area(self.line_2_az_list[-1], 
                                        self.dataset_in[self.line_2_id_list[-1]][2], self.dataset_in[self.line_2_id_list[-1]][3]):
                if not (self.local_index in self.forbidden_index_list):
                    self.temp_line_2_id_list.append(self.local_index)        
        if line == 3:
            if self.check_point_in_area(self.line_3_az_list[-1], 
                                        self.dataset_in[self.line_3_id_list[-1]][2], self.dataset_in[self.line_3_id_list[-1]][3]):
                if not (self.local_index in self.forbidden_index_list):
                    self.temp_line_3_id_list.append(self.local_index)

    def mean_az(self, input_list):
        n = len(input_list)
        if n > 4: n = 4
        sum = 0
        for i in range(n):
            sum = sum + input_list[ (-1-i) ]
        mean = sum / float(n)
        return mean

    def frontal_search_handler(self):
        #temp_forbidden = self.line_0_id_list + self.line_1_id_list + self.line_2_id_list + self.line_3_id_list
        #self.temp_line_0_id_list = [x for x in self.temp_line_0_id_list if x not in temp_forbidden]
        #self.temp_line_1_id_list = [x for x in self.temp_line_1_id_list if x not in temp_forbidden]
        #self.temp_line_2_id_list = [x for x in self.temp_line_2_id_list if x not in temp_forbidden]
        #self.temp_line_3_id_list = [x for x in self.temp_line_3_id_list if x not in temp_forbidden]
        if len(self.temp_line_0_id_list) > 0:
            ind = self.take_with_smaller_angle_declination(self.line_0_az_list[-1], self.dataset_in[self.line_0_id_list[-1]][2], 
                                                          self.dataset_in[self.line_0_id_list[-1]][3], self.temp_line_0_id_list)
            az = self.get_azimuth(self.dataset_in[self.line_0_id_list[-1]][2], self.dataset_in[self.line_0_id_list[-1]][3], 
                                self.dataset_in[ind][2], self.dataset_in[ind][3])

            if not (ind in self.line_0_id_list):
                self.line_0_id_list.append(ind)
                self.line_0_az_list.append(self.mean_az(self.line_0_az_list + [az]))

        if len(self.temp_line_1_id_list) > 0:
            ind = self.take_with_smaller_angle_declination(self.line_1_az_list[-1], self.dataset_in[self.line_1_id_list[-1]][2], 
                                                          self.dataset_in[self.line_1_id_list[-1]][3], self.temp_line_1_id_list)
            az = self.get_azimuth(self.dataset_in[self.line_1_id_list[-1]][2], self.dataset_in[self.line_1_id_list[-1]][3], 
                                self.dataset_in[ind][2], self.dataset_in[ind][3])
            if not (ind in self.line_1_id_list):
                self.line_1_id_list.append(ind)
                self.line_1_az_list.append(self.mean_az(self.line_1_az_list + [az]))
        if len(self.temp_line_2_id_list) > 0:
            ind = self.take_with_smaller_angle_declination(self.line_2_az_list[-1], self.dataset_in[self.line_2_id_list[-1]][2], 
                                                          self.dataset_in[self.line_2_id_list[-1]][3], self.temp_line_2_id_list)
            az = self.get_azimuth(self.dataset_in[self.line_2_id_list[-1]][2], self.dataset_in[self.line_2_id_list[-1]][3], 
                                self.dataset_in[ind][2], self.dataset_in[ind][3])
            if not (ind in self.line_2_id_list):
                self.line_2_id_list.append(ind)
                self.line_2_az_list.append(self.mean_az(self.line_2_az_list + [az]))
        if len(self.temp_line_3_id_list) > 0:
            ind = self.take_with_smaller_angle_declination(self.line_3_az_list[-1], self.dataset_in[self.line_3_id_list[-1]][2], 
                                                          self.dataset_in[self.line_3_id_list[-1]][3], self.temp_line_3_id_list)
            az = self.get_azimuth(self.dataset_in[self.line_3_id_list[-1]][2], self.dataset_in[self.line_3_id_list[-1]][3], 
                                self.dataset_in[ind][2], self.dataset_in[ind][3])
            if not (ind in self.line_3_id_list):
                self.line_3_id_list.append(ind)
                self.line_3_az_list.append(self.mean_az(self.line_3_az_list + [az]))
    
    def add_forbidden_elements(self):
        self.forbidden_index_list.append(self.line_0_id_list[-1])
        self.forbidden_index_list.append(self.line_1_id_list[-1])
        self.forbidden_index_list.append(self.line_2_id_list[-1])
        self.forbidden_index_list.append(self.line_3_id_list[-1])
        self.forbidden_index_list = list(set(self.forbidden_index_list))
    
    ##############################################################################################################

    def get_boundaries_iterator(self):
        self.find_next_point_calls_counter = self.find_next_point_calls_counter + 1
        ###
        if self.number_of_nearest_points/2 > self.main_index: floor = 0
        else: floor = self.main_index
        if self.main_index + self.number_of_nearest_points/2 > len(self.dataset_out): roof = len(self.dataset_out)
        else: roof = self.main_index + self.number_of_nearest_points/2
        return (floor, roof)

    def initial_horizontal_search(self):
        ###
        floor, roof = self.get_boundaries_iterator()

        #####<initial horizontal search>########
        for self.local_index in range(floor, roof):
            self.horizontal_initial_search_closest_lines() 
        self.horizontal_initial_search_take_closest_ones()
        for self.local_index in range(floor, roof):
            self.horizontal_initial_search_further_lines()
        self.horizontal_initial_search_take_closest_ones()
        self.add_forbidden_elements()
        self.init_frontal_search_base()
       

    def frontal_search(self):
        floor, roof = self.get_boundaries_iterator()
        while not self.stop_iterating:
            self.init_frontal_search_partial()
            amount_forbidden_elements_prev = len(self.forbidden_index_list)
            for self.local_index in range(floor, roof):
                for line in [0, 1, 2, 3]:
                    self.frontal_search_next_step(line)
        
            self.frontal_search_handler()
            self.add_forbidden_elements()

            amount_forbidden_elements_post = len(self.forbidden_index_list)

            if amount_forbidden_elements_post == amount_forbidden_elements_prev: 
                self.stop_iterating = True
        self.stop_iterating = False
        if self.main_index != len(self.dataset_out):
            self.main_index = roof
            self.next_iteration_over_available_points_flag = True
        else:
            self.main_index = 0
            self.next_iteration_over_available_points_flag = False


    def sort_lines(self):
        lines = []
        lines.append([0, self.line_0_id_list, len(self.line_0_id_list)])
        lines.append([1, self.line_1_id_list, len(self.line_1_id_list)])
        lines.append([2, self.line_2_id_list, len(self.line_2_id_list)])
        lines.append([3, self.line_3_id_list, len(self.line_3_id_list)])
        return sorted(lines,key=lambda x:x[2], reverse = True)

    def take_line(self):
        if self.line_to_take == 0:
            return (self.line_0_id_list, 0)
        if self.line_to_take == 1:
            return (self.line_1_id_list, 1)
        if self.line_to_take == 2:
            return (self.line_2_id_list, 2)
        if self.line_to_take == 3:
            return (self.line_3_id_list, 3)

    def get_az_list(self, id):
        if id == 0: return self.line_0_az_list
        if id == 1: return self.line_1_az_list
        if id == 2: return self.line_2_az_list
        if id == 3: return self.line_3_az_list


    def radian_side(self, side_string):
        chosen_id_list, id = self.take_line()
        chosen_az_list = self.get_az_list(id)
        if side_string == "LEFT": 
            add_radians = -np.pi/2.0
            out_id = id + 1
        elif side_string == "RIGHT": 
            add_radians = np.pi/2.0
            out_id = id - 1
        else: 
            print "second_processing_search - fatal ERROR"
            return 0
        if out_id < 0 or out_id > 3:
            print "second_processing_search - fatal id ERROR"
            return 0
        return add_radians, out_id, chosen_az_list, chosen_id_list

###############################################################################################################
    def second_processing_find_point(self, side_string):
        found_id_point = -1
        found_az_point = -1
        out_id = -1
        temp_id_list = []
        temp_az_list = []
        add_radians, out_id, chosen_az_list, chosen_id_list = self.radian_side(side_string)
        flag = True
        found_flag = False
        self.main_index = 0
        for i in range(len(chosen_id_list)):
            self.main_index = 0
            flag = True
            while flag:
                floor, roof = self.get_boundaries_iterator()
                for self.local_index in range(floor, roof):
                    if self.check_point_in_wide_area(chosen_az_list[i] + add_radians, self.dataset_out[chosen_id_list[i]][2], 
                                                self.dataset_out[chosen_id_list[i]][3]):
                        
                        if self.local_index in self.forbidden_index_list:
                            temp_id_list = []
                            temp_az_list = []
                            found_flag = False
                            break
                        else:                       
                            temp_id_list.append(self.local_index)
                            temp_az_list.append(chosen_az_list[i])
                            found_flag = True
                
                if self.main_index != len(self.dataset_out):
                    self.main_index = roof
                    flag = True
                else:
                    flag = False
            
            if found_flag:
                if temp_id_list[0] == 462: 
                    self.second_processing_flag = True
                    print 'impair'
                found_flag = False
                found_id_point = self.take_closest_one(self.dataset_out[chosen_id_list[i]][2], 
                                                    self.dataset_out[chosen_id_list[i]][3], temp_id_list)
#####################
                if  found_id_point == -1:
                    print "CRASH"
                    print temp_id_list
                    print "---"
                    for j in range(len(temp_id_list)):
                        print chosen_id_list[i]
                        print self.get_distance(self.dataset_out[chosen_id_list[i]][2], self.dataset_out[chosen_id_list[i]][3],self.dataset_out[temp_id_list[j]][2],self.dataset_out[temp_id_list[j]][3])
                        print self.check_point_in_wide_area(chosen_az_list[i] + add_radians, self.dataset_out[temp_id_list[j]][2], 
                                                self.dataset_out[temp_id_list[j]][3])
                    print '---'
                    break
#####################
                found_az_point = temp_az_list[temp_id_list.index(found_id_point)]
                print i, 'index found_flag=True', temp_id_list
                break
        print (found_id_point, found_az_point, out_id), 'output'
        return (found_id_point, found_az_point, out_id)

################################################################################################


    def second_processing_append_point(self, found_id_point, found_az_point, out_id):
        if out_id == 0:
            self.line_0_id_list.append(found_id_point)
            self.line_0_az_list.append(found_az_point)
        elif out_id == 1:
            self.line_1_id_list.append(found_id_point)
            self.line_1_az_list.append(found_az_point)
        elif out_id == 2:
            self.line_2_id_list.append(found_id_point)
            self.line_2_az_list.append(found_az_point)
        elif out_id == 3:
            self.line_3_id_list.append(found_id_point)
            self.line_3_az_list.append(found_az_point)
        else:
            print "second_processing_append_point - FATAL ERROR ---------------------------"
            return 0
        self.add_forbidden_elements()
            
#######################################################################################################
parser = argparse.ArgumentParser()
parser.add_argument('-lon', '--lon', help='INITIAL_GPS_LON', type=float)
parser.add_argument('-lat', '--lat', help='INITIAL_GPS_LAT', type=float)
parser.add_argument('-heading', '--heading', help='INITIAL_GPS_HEADING', type=float)
parser.add_argument('-lane', '--lane', help='LANE', type=int)
args = parser.parse_args()
MappingPostprocessor.INITIAL_GPS_LON = args.lon
MappingPostprocessor.INITIAL_GPS_LAT = args.lat
MappingPostprocessor.INITIAL_GPS_HEADING = args.heading
MappingPostprocessor.INITIAL_GPS_LANE = args.lane
#######################################################################################################
MP = MappingPostprocessor()
print MappingPostprocessor.INITIAL_GPS_LANE
MP.get_dataset()
MP.clear_ids()
MP.initial_horizontal_search()
MP.floor_limit_sidelook = 4
MP.angle_range_limit_sidelook = 120 * np.pi/180.0
MP.distance_limit_sidelook = 14
while MP.next_iteration_over_available_points_flag:
    MP.frontal_search()

for _ in range(6):
    line_queue = [MP.sort_lines()[i][0] for i in range(4)]
    for line in range(4):
        MP.next_iteration_over_available_points_flag = True
        MP.line_to_take = line_queue[line]
        stop_main_iteration = False
        stop1 = False
        stop2 = False
        while not stop_main_iteration:
            ### second processing
            if MP.line_to_take == 3: stop2 = True
            if MP.line_to_take == 0: stop1 = True 

            if not stop1:
                found_id_point, found_az_point, out_id = MP.second_processing_find_point("RIGHT")
                if found_id_point != -1:    
                    MP.second_processing_append_point(found_id_point, found_az_point, out_id)
                    MP.main_index = 0
                    MP.next_iteration_over_available_points_flag = True
                    while MP.next_iteration_over_available_points_flag:
                        MP.frontal_search()
                else:
                    stop1 = True
            ###
            if not stop2:
                found_id_point, found_az_point, out_id = MP.second_processing_find_point("LEFT")
                if found_id_point != -1:    
                    MP.second_processing_append_point(found_id_point, found_az_point, out_id)
                    MP.main_index = 0
                    MP.next_iteration_over_available_points_flag = True
                    while MP.next_iteration_over_available_points_flag:
                        MP.frontal_search()
                else:
                    stop2 = True
                ###
            print stop1, stop2
            stop_main_iteration = stop1 and stop2
    print "NEEEXT"


MP.refract_ids()
MP.save_dataset()
#print MP.sort_lines()

print [MP.line_0_id_list, '0']
print [MP.line_1_id_list, '1']
print [MP.line_2_id_list, '2']
print [MP.line_3_id_list, '3']

