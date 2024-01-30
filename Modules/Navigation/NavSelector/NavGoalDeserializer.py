from homodeus_precomp import List, Point
from NavGoal import NavGoal
import json

# TODO Must be rename class
class NavGoalDeserializer :
    __fileName : str = "/home/pal/tiago_public_ws/src/pmb2_simulation/pmb2_2dnav_gazebo/script/NavSelector/predefNavGoal.json"
    __keyPosX : str = "PosX"
    __keyPosY : str = "PosY"
    __keyPosZ : str = "PosZ"
    __keyObjOri : str = "ObjOri"

    def __init__(self, fileName : str = None) -> None:
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

    def Write(self, navGoals : List[NavGoal] = []) -> None :
        with open(self.__fileName,"w+") as file :
            json.dump(self.__SerializeNavGoal(navGoals), file, indent=4)
            
    #Deserializes the Json data for a name and returns a new NavGoal
    def __DeserializeNavGoal(self, jsonData, name : str) -> NavGoal:
        return NavGoal(
            jsonData[name][self.__keyPosX],
            jsonData[name][self.__keyPosY],
            jsonData[name][self.__keyPosZ],
            jsonData[name][self.__keyObjOri],
            name
            #,jsonData[name]["ShouldStart"]
        )

    #Serializes the NavGoal
    def __SerializeNavGoal(self, navGoals : List[NavGoal]) -> dict:
        jsonData : dict = {}
        for navGoal in navGoals:
            pos : Point = navGoal.GetPoint()
            orientation : float = navGoal.GetOriScalar()
            name: str = navGoal.GetName()
            jsonData[name] = {self.__keyPosX : pos.x, self.__keyPosY : pos.y, self.__keyPosZ : pos.z, self.__keyObjOri : orientation}
        return jsonData
