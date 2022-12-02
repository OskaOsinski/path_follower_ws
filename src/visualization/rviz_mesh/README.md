Paczka wyświetla teksturę znajdującą się w katalogu meshes/gtc_map/gtc_map.dae.
Dodatkowo w tym samym katalogu dae znajduje się folder vehicle z modelem Kia Niro.

Uruchomienie wizualizacji modelu w rvizie:
roslaunch rviz_mesh display.launch

Zmiana pliku urdf, znajdującego się w folderze urdf, nie wymusza ponownej kompilacji. 
Nazwa topica, na który nadawana jest mapa, to gtc_map (linia 13, 29 i 37 w pliku *.dae).
Plik display.launch uruchamia także skonfigurowanego rviza, dla domyślnego topicu, czy wypełnienia tekstur.
W razie zmiany nadawanego topicu i próby odtworzenia poprzez display.launcha mapy ponownie, należy zmienić występujące (w katalogu rviz i dwóch plikach znajdujących się w nim) w nim domyślne ustalone nazwy topicu (gtc_map) na wybrany.

Model Kia Niro nie jest wyświetlany. Można go pokazać, odkomentowując w pliku ./urdf/gtc_map.urdf.xacro blok modelu, który znajduje się w katalogu meshes/vehicle/kia.dae.

Tekstury toru ignorują światło pochodzące z rviza (domyślne oświetlenie pochodzące od kamery) i są równomiernie oświetlone.


