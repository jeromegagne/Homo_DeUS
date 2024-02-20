Suivre instructions pour 20.04 - http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS ROS installation

ROS installation
ROS packages installation
Source-based installation
Packages ajoutés (avec instructions)

Exemple d'utilisation du module en simulation : 

1. Identifier le module sur lequel vous voulez simuler (par exemple pmb2_2dnav_gazebo)
2. Créer un fichier scripts et ajouter le NavSelector
3. Ajouter un MAKEFILE et ajouter comme exécutable le NavSelector
    - Donner les permissions au NavSelector
4. Lancer la simulation
5. Lancer le module en utilisant rosrun
# NavSelector

# Class Variables

## Not currently used

- staticTopic = "navigationTopic"
- staticInterruptTopic = "navigationInterrupt"

## Used at the moment

### Private

- __goalList : List[NavGoal]
- __currentGoal : NavGoal
- __callBack = None (Supposed to be a Function)
- __currentLocation : Tuple[Point,Quaternion]

# Class Methods

## Python Class Method override

```python
__init__(self, goalList : List[NavGoal] = [], currentGoal : NavGoal = None, topic : String = None) -> None
```

### Function Explanation

- Class constructor of the NavigationSelector, can be created with default value or specified values.
- Creates a [NavGoalDeserializer](https://www.notion.so/NavGoalDeserializer-95c472c1c47a488187a42921d07e5cfd?pvs=21) to get Goals to use from the Json in the file.
- Fetches the current position on load from "moveBaseActionFeedback.base_position.pose".
- Returns nothing.

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goalList | List[NavGoal] | [] | List of goal that were created before the class |
| currentGoal | Navgoal | None | First goal that we want to force |
| topic | String | None | The topic that we want to use [UNUSED FOR THE MOMENT] |

 

## Class methods

### Public methods

### ConnectCallBack

```python
ConnectCallBack(self,callBackFunction) -> None
```

### Function Explanation

- Connects the NavSelector to a single function passed by the controller
- Returns nothingGetGoalList(self) -> List[NavGoal]

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| callBackFunction | Function [Not enforced as I was not sure how to type it] | No default param | A CallBack function taking any function taking a parameter |

### GetGoalList

```python
GetGoalList(self) -> List[NavGoal]
```

### Function Explanation

- Returns a copy of the goalList

### GetCurrentGoal

```python
GetCurrentGoal(self) -> NavGoal
```

### Function Explanation

- Return the goal that will be processed

### SetCurrentGoal

```python
SetCurrentGoal(self, goal : NavGoal) -> None
```

### Function Explanation

- Sets the current goal to another one
- Discards the current goal
- Returns Nothing
- [Maybe should add a boolean param to see if we want to keep the goal that we currently discard]

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goal | NavGoal | No default value | The goal we wish to apply |

### AddGoal

```python
AddGoal(self, goal : NavGoal) -> None
```

### Function Explanation

- Adds a goal to the goal list
- No special insertion, so the list is currently unsorted
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goal | NavGoal | No default value | The goal we wish to append to the goal list |

### AddGoal

```python
AddGoal(self, goalX : float, goalY : float, goalZ : float, goalOri : float, name : String) -> None
```

### Function Explanation

- Creates and adds a goal to the goalList
- Create the goal using the parameters passed
- For more precisions on the constructor, see [NavSelector](https://www.notion.so/NavSelector-cf58eced786747e793e5cd1ef96abaea?pvs=21)

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goalX  | Float | No default value | The X position of the goal |
| goalY | Float | No default value | The Y position of the goal |
| goalZ  | Float | No default value | The Z position of the goal |
| goalOri  | Float | No default value | The orientation of the goal |
| name | Float | No default value | The name of the goal |

### ExtendGoals

```python
ExtendGoals(self, goals : List[NavGoal]) -> None
```

### Function Explanation

- Adds a list of goal to the goal list
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| goals | List[NavGoal] | No default values | The list of goal we wish to add to the goal list |

### RemoveGoal

```python
RemoveGoal(self, index : int) -> None
```

### Function Explanation

- Remove the goal at the specified index
- Currently, no check if it is possible [Could cause problems]
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| index | int | No default value | Index is currently not checked |

### RemoveCurrentGoal

```python
RemoveCurrentGoal(self) -> None
```

### Function Explanation

- Removes the current goal and tries to get an unblocked goal to replace the now vacant current goal
- Checks sequentially if the goals are blocked
    - If one is found, then we : set the current goal, remove the goal in the goal list, sets the current pose (position and angle in `Point` and `Quaternion` respectively) and returns.
    - If none is found, then we : set the current goal to the `None` and we send a warning.
- Returns nothing

### BlockGoal

```python
BlockGoal(self, index : int) -> None
```

### Function Explanation

- Blocks the goal at the specified index, so that we won’t execute it if it unless we unblock it
- Checks if the index is coherent
    - If the index is coherent, then we : block it and return
    - If the index is not coherent, then we send a warning
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| index | int | No default value | Index to block |

### BlockAllGoals

```python
BlockAllGoals(self) -> None
```

### Function Explanation

- Blocks all goal in the list, but won’t affect the current goal
- Returns nothing

### UnblockGoal

```python
UnblockGoal(self, index : int) -> None
```

### Function Explanation

- Unblocks the goal at the specified index
- Checks if the index is coherent
    - If the index is coherent, then we : block it and return
    - If the index is not coherent, then we send a warning
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| index | int  | No default value | Index to unblock |

### UnblockAllGoals

```python
UnblockAllGoals(self) -> None
```

### Function Explanation

- Unblocks all goals in the list, but won’t affect the current goal
- Returns nothing

### NbUnblockedTask

```python
NbUnblockedTask(self) -> int
```

### Function Explanation

- Checks all goals in the goal list to count the number of unblocked task
- Returns the count

### run

```python
run(self) -> None
```

### Function Explanation

- Initalize the actionlib Client
- Waits for the server to be ready
- Sets class variables to be valid (if available)
- Display menu and calls functions upon selection
- Returns nothing

### Private Methods

### __OnNavGoalFail

```python
__OnNavGoalFail(self, errorDesc : String, goalStatus : GoalStatus) -> None
```

### Function Explanation

- Function decorator that will throw an error [non-blocking] to the controller using a call to the `__OnEvent(eventContent)`
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| errorDesc | Description of the error | No default value | Description associated to the error |
| goalStatus | GoalStatus | No default value | GoalStatus from rospy, useful to know what failed |

### __OnNavGoalSuccess

```python
__OnNavGoalSuccess(self) -> None
```

### Function Explanation

- Function decorator that will print the success of the goal and send an event to the controller using a call to the `__OnEvent(eventContent)`
- Returns nothing

### __OnEvent

```python
__OnEvent(self, eventContent) -> None
```

### Function Explanation

- Sends an event to the controller using the callBack that is connected, prints that an event was sent
- Won’t do anything if the eventContent is None and if we don’t have a current goal [Should still be the same as this function is called *before* the goal is changed]
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| eventContent | Any | No default value | The content that we want to transmit to the controller |

### Sort

```python
Sort(self) -> None
```

### Function Explanation

- Unimplemented for now
- Should sort the goal list by key(s)
- Returns nothing

### __sendGoal

```python
__sendGoal(self) -> None
```

### Function Explanation

- Sends the goal to the predefined topic
    - Checks for coherence of the current goal then : updates the goal in the `MoveBaseGoal`, on completion of the goal we send events to the controller whether or not the goal has succeeded
    - If there are no current goal and unblocked task, will warn the controller
    - If there are no current goal and no goal in the goal list, will warn the controller
- Returns nothing

### __display_menu

```python
__display_menu(self) -> None:
```

### Function Explanation

- Show the debug menu with options to select

### __OnEvent

```python
__OnEvent(self, eventContent) -> None
```

### Function Explanation

- Sends an event to the controller using the callBack that is connected, prints that an event was sent
- Won’t do anything if the eventContent is None and if we don’t have a current goal [Should still be the same as this function is called *before* the goal is changed]
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| eventContent | Any | No default value | The content that we want to transmit to the controller |

### __OnEvent

```python
__OnEvent(self, eventContent) -> None
```

### Function Explanation

- Sends an event to the controller using the callBack that is connected, prints that an event was sent
- Won’t do anything if the eventContent is None and if we don’t have a current goal [Should still be the same as this function is called *before* the goal is changed]
- Returns nothing

### Parameters explanation

| Parameter | Type | Default Value | Note |
| --- | --- | --- | --- |
| eventContent | Any | No default value | The content that we want to transmit to the controller |