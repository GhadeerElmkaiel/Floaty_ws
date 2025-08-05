import sys
import subprocess


cmd = "python3 python_script.py"

def main(arg1, arg2):
    cmd_edited = cmd + " " + str(arg1)+ " " + str(arg2)
    p = subprocess.Popen(cmd_edited, stdout = subprocess.PIPE, shell = True)
    out, error = p.communicate()
    # result = out.split('\n')
    # for lin in result:
    #     if not lin.startswith('#'):
    #         print(lin)
    pass


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])