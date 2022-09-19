import yaml
import os
import sys

# Common variables
output_file_name = "/tmp/ros_source_file.txt"
shell = os.popen(
    "echo $SHELL | awk -F '/' '{print $NF}'").readlines()[0].rstrip("\n")

# Read YAML file.
f = open('%s' % sys.argv[1], 'r')
if yaml.__version__ >= '5.1.0':
    source_file = yaml.load(f, Loader=yaml.FullLoader)
else:
    source_file = yaml.load(f)
f.close()

if (source_file['Config']['menu_enable'] != True):
    sys.exit(0)

# Generate Menu
keys = list(source_file['Menu'])
print('************ Neuron Startup Menu for ROS *************')
print('* Usage: To set ROS env to be auto-loaded, please    *')
print('*        assign ros_option in ros_menu/config.yaml   *')
print('******************************************************')
print('0) Do nothing')
choose_dict = {}
for key in keys:
    if str(source_file['Menu'][key]['option_num']) in list(choose_dict):
        raise SyntaxError('Some option numbers(option_num) in the YAML file are duplicated.')
        sys.exit(0)
    print('%s) %s ' % (source_file['Menu'][key]['option_num'], key))
    choose_dict['%s' % source_file['Menu'][key]['option_num']] = key

# Choose Menu
if source_file['Config']['ros_option'] != 'menu':
    ros_option = str(source_file['Config']['ros_option'])
    print('Please choose an option: default=%s' % source_file['Config']['ros_option'])
    print('------------------------------------------------------')
else:
    try:
        ros_option = input('Please choose an option: ')
    except KeyboardInterrupt:
        ros_option = ''
        print('')
    print('------------------------------------------------------')
    if len(ros_option) == 0 or ros_option == '0' or ros_option not in list(choose_dict):
        print('Do nothing!')
        sys.exit(0)

choose = choose_dict[ros_option]


def read_cmds():
    ret_cmds = ""
    if (source_file['Menu'][choose]['cmds'] is not None):
        for cmds in source_file['Menu'][choose]['cmds']:
            ret_cmds += cmds + '\n'
    return ret_cmds


def source_ros1():
    current_ip = os.popen("hostname -I | awk '{print $1}'").readlines()[0]
    if (len(current_ip) == 0):
        current_ip = '127.0.0.1'
    ros_master_uri = source_file['Menu'][choose]['master_ip']
    if (ros_master_uri is None):
        ros_master_uri = current_ip
    source_ros = 'source %s/setup.%s' % (
        source_file['Menu'][choose]['ros1_path'], shell)
    export_ip = 'export ROS_IP=%s' % current_ip
    export_ros_master_uri = 'export ROS_MASTER_URI=http://%s:11311' % ros_master_uri.rstrip("\n")
    print('* ROS_IP=%s' % current_ip.rstrip('\n'))
    print('* ROS_MASTER_URI %s' % ros_master_uri.rstrip('\n'))
    print('------------------------------------------------------')
    return source_ros + '\n' + export_ros_master_uri + '\n' + export_ip + '\n'


def source_ros2():
    ros_domain_id = source_file['Menu'][choose]['domain_id']
    if (ros_domain_id is None):
        ros_domain_id = int(source_file['Config']['default_ros_domain_id'])
    source_ros = 'source %s/local_setup.%s' % (source_file['Menu'][choose]['ros2_path'], shell)
    source_colcon = 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.%s' % shell
    export_domain_id = 'export ROS_DOMAIN_ID=%d' % ros_domain_id

    python3_path_setup = "alias python3='/usr/bin/python3'"
    python_path_setup = "alias python='/usr/bin/python3'"

    print('* ROS_DOMAIN_ID = %d' % ros_domain_id)
    print('------------------------------------------------------')
    return source_colcon + '\n' + source_ros + '\n' + export_domain_id + '\n' + python3_path_setup + '\n' + python_path_setup + '\n'


def check_bridge():
    if ((os.system('ls %s/lib/ | grep -q ros1_bridge' % source_file['Menu'][choose]['ros2_path']) >> 8) == 1):
        print('You need to install ros1_bridge first.')
        print('Installation command: sudo apt install ros-%s-ros1-bridge' % source_file['Menu'][choose]['ros2_version_name'])
        return False
    return True


ros_source_file = open(output_file_name, 'w')
ros_source_file.write("shell=`cat /proc/$$/cmdline | tr -d '\\0' | tr -d '-'`\n")
ros_source_file.write("PS1=\"(%s) $PS1\"\n" % choose)
if (source_file['Menu'][choose]['ROS_version'] == 1):
    ros_source_file.write(source_ros1()+read_cmds())
if (source_file['Menu'][choose]['ROS_version'] == 2):
    ros_source_file.write(source_ros2()+read_cmds())
if (source_file['Menu'][choose]['ROS_version'] == 'bridge' and check_bridge()):
    ros_source_file.write(source_ros1()+source_ros2()+read_cmds())
ros_source_file.close()
