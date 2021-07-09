from gmplot import gmplot
import math

#Read data to list
file_name = raw_input('gps data file name: ')

with open(file_name + '.txt', 'rU') as file:
    lines = file.read()
    gps_data = lines.split('\n')

# spilt gps to latitude and longtitude
latitude = []
longtitude = []
for data in gps_data:
    gps_split = data.split(" ")
    if (len(gps_split) == 4):
        
        # change to GPS DD
        '''
        latitude_tmp = math.floor( gps_split[3] /100)
        latitude_tmp += (gps_split[3] - latitude_tmp * 100) / 60.0
        longitude_tmp = math.floor( gps_split[5] /100)
        longitude_tmp += (gps_split[5] - longitude_tmp * 100) / 60.0
        '''

        latitude.append(float(gps_split[0]))
        longtitude.append(float(gps_split[1]))

# Place map
gmap = gmplot.GoogleMapPlotter(latitude[0], longtitude[0], 17)        
gmap.scatter(tuple(latitude), tuple(longtitude), '#3B0B39', size=1, marker=False)

# Draw and save
gmap.draw(file_name + "_googlemap.html")