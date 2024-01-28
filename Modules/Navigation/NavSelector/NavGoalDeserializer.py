from homodeus_precomp import String, List
from NavGoal import NavGoal
import json

class NavGoalDeserializer :
    __fileName : String = "/home/pal/tiago_public_ws/src/pmb2_simulation/pmb2_2dnav_gazebo/script/NavSelector/predefNavGoal.json"

    def __init__(self, fileName : String = None) -> None:
        if fileName is not None :
            self.__fileName = fileName
    
    #Will read the json and call the callback function with the newly read items (like add the items to an array) 
    def Read(self, callback) -> None :
        resList : List[NavGoal] = []
        with open(self.__fileName,"r") as file :
            jsonData = json.load(file)
        for name in jsonData :
            resList.append(self.__DeserializeNavGoal(jsonData, name))
        callback(resList)

    #Deserializes the Json data for a name and returns a new NavGoal
    def __DeserializeNavGoal(self, jsonData, name : String) -> NavGoal:
        return NavGoal(
            jsonData[name]["PosX"],
            jsonData[name]["PosY"],
            jsonData[name]["PosZ"],
            jsonData[name]["ObjOri"],
            name
            #,jsonData[name]["ShouldStart"]
        )