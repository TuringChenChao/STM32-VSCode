import subprocess
import select

if __name__ == '__main__':
    f = open('gdb_output.txt','a+')
    # gdb = subprocess.Popen(['arm-none-eabi-gdb'],stdin=subprocess.PIPE, stdout=f, universal_newlines=True, shell=True)
    # gdb = subprocess.Popen(['arm-none-eabi-gdb'], bufsize=0,stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True, shell=False)
    gdb = subprocess.Popen(['arm-none-eabi-gdb'], bufsize=0,stdin=subprocess.PIPE, stdout=f, universal_newlines=True, shell=False)
    # gdb = subprocess.Popen(['arm-none-eabi-gdb'], bufsize=0,stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=False)
    # f.flush()
    # f.seek(2)
    # f.write('file ./out/HAL_demo.elf\n')
    # f.flush()

    # gdb.communicate(input='file ./out/HAL_demo.elf\n')
    # gdb.communicate(input='info b\n')
    # gdb.communicate(input='ptype UART_HandleTypeDef\n')
    f.flush()
    f.close()
    # f.write('file ./out/HAL_demo.elf\n')
    # f.flush()

    f = open('gdb_output.txt','a+')
    f.write('file ./out/HAL_demo.elf\n')
    f.flush()
    f.close()
    f = open('gdb_output.txt','a+')
    gdb.stdin.write('file ./out/HAL_demo.elf\n')
    gdb.stdin.flush()
    f.flush()
    f.close()

    f = open('gdb_output.txt','a+')
    f.write('info b\n')
    f.flush()
    f.close()
    f = open('gdb_output.txt','a+')
    gdb.stdin.write('info b\n')
    gdb.stdin.flush()
    f.flush()
    f.close()

    f = open('gdb_output.txt','a+')
    f.write('ptype UART_HandleTypeDef\n')
    f.flush()
    f.close()
    f = open('gdb_output.txt','a+')
    gdb.stdin.write('ptype UART_HandleTypeDef\n')
    gdb.stdin.flush()
    f.flush()
    f.close()

    f = open('gdb_output.txt','a+')
    f.write('quit\n')
    f.flush()
    f.close()
    f = open('gdb_output.txt','a+')
    gdb.stdin.write('quit\n')
    gdb.stdin.flush()
    f.flush()
    f.close()

    # # f.seek(2)
    # # f.write('info b\n')
    # # f.flush()
    # gdb.stdin.write('info b\n')
    # gdb.stdin.flush()
    # f.flush()

    # # txt_list = []
    # # while True:
    # #     txt = gdb.stdout.read(1)
    # #     if txt != '':
    # #         txt_list.append(txt)
    # #     else:
    # #         break
    # # # gdb.stdin.flush()
    # # # txt = gdb.communicate(input= 'info b\n')

    # # # f.seek(2)
    # # # f.write('ptype UART_HandleTypeDef\n')
    # # # f.flush()
    # gdb.stdin.write('ptype UART_HandleTypeDef\n')
    # gdb.stdin.flush()
    # f.flush()

    # gdb.stdin.write('quit\n')
    # gdb.stdin.flush()
    # f.flush()

    # f.close()