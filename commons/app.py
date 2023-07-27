from os import listdir, walk,chmod
import PySimpleGUI as sg
import platform

workspace = "./tiago_public_ws/"
src = "./tiago_public_ws/src/"

path_to_maps = "./maps/" if platform.system() != "Linux" else ".pal/tiago_maps/configurations/"
dir_list = listdir(src)
map_list = listdir(path_to_maps)
sub_module_list = []

module_list_visible = True
module_text = sg.Text("What module do you want to launch")
module_combo_box = sg.Combo(dir_list,default_value=dir_list[0],key='module_name',enable_events=module_list_visible,size=(30,10))

sub_module_list_visible = True
sub_module_text = sg.Text("Which sub-module do you want to launch")
sub_module_combo_box = sg.Combo(dir_list,default_value="",key='sub_module_combo',enable_events=module_list_visible,size=(30,10))

launch_list_text = sg.Text("Which script do you want to launch ")
launch_list_combo = sg.Combo(values=[], default_value="",key="launch_combo",size=(30,10))
check_public_sim = sg.Checkbox("Is public sim?",key="check_public_sim",default=True,enable_events=True)

map_list_text = sg.Text("What map will we run this on          ")
map_list_combo = sg.Combo(values=map_list, default_value="",key="map_combo",enable_events=True,size=(30,0))

input_sup_text = sg.Text("Write other args that are necessary")
input_sup = sg.Input("",key="arg_sup")

file_name_text = sg.Text("File name")
file_name = sg.Input("",key="filename")

ok_button = sg.Button('Ok',key="ok",enable_events=True)
preview_button = sg.Button("Preview of command",key="preview",enable_events=True)
preview_text = sg.Text("",key="preview_text",text_color="white",enable_events=True)

def find_launch_file(value:str):
    res = []
    for _, __, files in walk(f"{src}{value}"):
        for file in files:
            if file.endswith(".launch"):
                res.append(file)
    return res

def generate_command(values):
    world = ""
    mapp = ""
    if value['map_combo'] != '':
        path_to_maps = "$HOME/.pal/tiago_maps/configurations/" if platform.system() == "Linux" else "./maps/"
        mapp = f"map:={path_to_maps}{value['map_combo']}"
        world = f"world:={value['map_combo']}"
    return f"roslaunch {value['sub_module_combo']} {value['launch_combo']} public_sim:={value['check_public_sim']} {mapp} {world} {value['arg_sup']}"


layout = [  [module_text,module_combo_box],     
            [sub_module_text,sub_module_combo_box],     
            [launch_list_text,launch_list_combo],
            [map_list_text,map_list_combo],
            [check_public_sim],
            [input_sup_text,input_sup],
            [file_name_text,file_name],
            [ok_button,preview_button,preview_text] ]

# Create the window
window = sg.Window('Tiago Command Generetor V0.0.1', layout)      # Part 3 - Window Defintion
sub_module_elem = window["sub_module_combo"]
launch_elem = window["launch_combo"]
preview_elem = window["preview_text"]

while True:
    event,value = window.read()
    if event in (sg.WIN_CLOSED,'Exit'):
        break
    elif event == 'module_name':
        sub_module_elem.update(values=listdir(f"{src}{value['module_name']}"))
    elif event == 'sub_module_combo':
        launch_elem.update(values=find_launch_file(f'{value["module_name"]}/{value["sub_module_combo"]}'))
    elif event == 'preview':
        preview_elem.update(generate_command(value))
    elif event =='ok':
        file_path = f'{"./" if platform.system() != "Linux" else ""}{value["filename"]}.sh'
        file = open(file_path,'w+')
        file.write(f"#! /usr/bin/bash\ncd {workspace}\nsource ./devel/setup.bash\n{generate_command(value)}")
        file.close()
        if platform.system() == "Linux":
            chmod(file_path,0o755)
        break

window.close()


