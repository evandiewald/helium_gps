import http.client
import numpy as np
import gmplot

conn = http.client.HTTPSConnection('api.pipedream.com')
conn.request("GET", '/v1/sources/dc_YGulKQ/event_summaries?expand=event', '', {
  'Authorization': 'Bearer c968c4dc5acdc9d9785a84222793e2d8',
})

res = conn.getresponse()
data = res.read()

decoded = data.decode("utf-8")
d = eval(decoded)
data = d['data']

alt = np.zeros((len(data),))
lat = np.zeros((len(data),))
lon = np.zeros((len(data),))
for i in range(len(data)):
    try:
        event_data = data[i]['event']['body']['decoded']['payload'][0]['value']
        lat[i] = event_data['latitude']
        lon[i] = event_data['longitude']
        alt[i] = event_data['altitude']
    except:
        continue

# GoogleMapPlotter return Map object
# Pass the center latitude and
# center longitude
gmap1 = gmplot.GoogleMapPlotter(lat[0],
                                lon[0], 13)
gmap1.scatter(lat[0:5], lon[0:5], '#FF0000',
                                size = 400, marker = False)
# Pass the absolute path
gmap1.draw("map.html")
