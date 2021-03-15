    var map;
    const options = {
        title: 'Time Taken for items to be Shipped',
        hAxis: { title: 'Order ID'},
        vAxis: { title: 'Time Taken (s)'},
        legend: { position: "none" },
      };
    
    /////////////Ajax Requests////////////
    $(document).ready(function() {
        // Fetch the initial Map
        refreshMap();
        //Initializing Map
        map = L.map('map').setView([20.5937, 78.9629], 4);
        map.touchZoom.disable();
        map.doubleClickZoom.disable();
        map.scrollWheelZoom.disable();
        // Fetch every 2 second
        setInterval(refreshMap, 3000);
    });

    google.charts.load("current", {packages:['corechart']});
    google.charts.setOnLoadCallback(refreshMap);

    function refreshMap(){
        var container = L.DomUtil.get('map');

      	if(container != null){
        container._leaflet_id = null;
        }

        var jsonDataObject =[];
        var trHTML = '';
        var graph_arr = [['Order ID', 'Time Taken', { role: 'style' }]];

        const MarkersList= {
            "HP":'#FF0000',
            "MP":'#FFFF00',
            "LP":'#00FF00'
        }

        var currIcon = new L.icon({iconSize: [30, 30]});
        
    //$.getJSON('https://spreadsheets.google.com/feeds/list/1-l6pXtbA0sthpoBi0tQ5VfJExs3Yo6CyNsjqJZwVGcg/1/public/full?alt=json', function(data) { //This is for my personal sheet
    $.getJSON('https://spreadsheets.google.com/feeds/list/1hwNalvg0eCn6meOGQayI29JwElJbutMHauXlzonansU/5/public/full?alt=json', function(data) { //e-Yantra Sheet
        
        var trHTML = '';
        //This loop iterates over all the rows of sheet   
        for (var i = 0; i < data.feed.entry.length; ++i) { //data.feed.entry.length == Rows of sheet

            var json_data = {
                "teamID": "VB#0000",
                "uniqueID" : "1234",
                "orderID" : data.feed.entry[i].gsx$orderid.$t,
                "item" : data.feed.entry[i].gsx$item.$t,
                "priority": data.feed.entry[i].gsx$priority.$t,
                "city" : data.feed.entry[i].gsx$city.$t,

                "Longitude": parseFloat(data.feed.entry[i].gsx$longitude.$t),
                "Latitude": parseFloat(data.feed.entry[i].gsx$latitude.$t),
            
                "orderdispatched": data.feed.entry[i].gsx$orderdispatched.$t,
                "ordershipped" : data.feed.entry[i].gsx$ordershipped.$t,
                
                "ordertime" : data.feed.entry[i].gsx$orderdatetime.$t,
                "dispatchedtime": data.feed.entry[i].gsx$dispatchdatetime.$t,
                "shippingtime" : data.feed.entry[i].gsx$shippeddatetime.$t,
                "timetaken" : data.feed.entry[i].gsx$timetaken.$t
            };

           
            // for (const vals in json_data) {
            // console.log(`${vals}: ${json_data[vals]}`);
            // }

            //Pushing on row at a time
            jsonDataObject.push(json_data);
            graph_arr.push([json_data["orderID"],parseInt(json_data["timetaken"]),MarkersList[json_data["priority"]]])

            }


            // console.log(graph_arr)
            
            //iterating over contents of individual row
            for (var j = 0; j < jsonDataObject.length; j++) {
                
                if (jsonDataObject[j].orderdispatched == 'YES' && jsonDataObject[j].ordershipped == 'YES'){
                    currIcon.options.iconUrl = "marker_green.png";
                } 
                else if (jsonDataObject[j].orderdispatched == 'YES'){
                    currIcon.options.iconUrl = "marker_yellow.png";
                }
                else{
                    currIcon.options.iconUrl = "marker_red.png";
                }
                
                
                //Plots marker on corresponding latitude and longitude
                var marker = L.marker(L.latLng(parseFloat(jsonDataObject[j].Latitude), parseFloat(jsonDataObject[j].Longitude)),{icon: currIcon});
            
                //Attach a popup with name of the city
                marker.bindPopup(jsonDataObject[j].city, {autoClose: false});

                map.addLayer(marker);

                //Attach onclickListner for that marker
                marker.on('click', onClick_Marker)

                // Attach the corresponding JSON data to your marker:
                marker.myJsonData =jsonDataObject[j];
        
                function onClick_Marker(e) {
                        var marker = e.target;

                        popup = L.popup()
                        .setLatLng(marker.getLatLng())
                        .setContent("Order ID: " + marker.myJsonData.orderID + " || Dispatched: " +   marker.myJsonData.orderdispatched+ " || Dispatched: " +   ((marker.myJsonData.ordershipped=='YES')  ? 'YES' : 'NO'))
                        .openOn(map);
                    }
           
                //Add map inside the div tag 
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                        attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
                    
                }).addTo(map);
                
                //Table
                trHTML += '<tr><td>' + jsonDataObject[j]["orderID"] + 
                          '</td><td>' + jsonDataObject[j]["item"] + 
                          '</td><td>' + jsonDataObject[j]["priority"] + 
                          '</td><td>' + jsonDataObject[j]["city"] + 
                          '</td><td>'  + jsonDataObject[j]["orderdispatched"] + 
                          '</td><td>'  + jsonDataObject[j]["ordershipped"] + 
                          '</td><td>'  + jsonDataObject[j]["ordertime"] + 
                          '</td><td>'  + jsonDataObject[j]["dispatchedtime"] + 
                          '</td><td>'  + jsonDataObject[j]["shippingtime"] + 
                          '</td><td>'  + jsonDataObject[j]["timetaken"] +
                          '</td></tr>';                   
                //console.log(trHTML);
                $('#tableContent').html(trHTML);
                
                }
                
//------------------------
                var graphArray_Final = google.visualization.arrayToDataTable(graph_arr);
    
                var data = new google.visualization.DataView(graphArray_Final); 

                  var chart = new google.visualization.ColumnChart(document.getElementById('column_chart'));
                  chart.draw(data, options);
//-----------------------------


        });
    }
