import sys
import os


slot_space_info_writePath = "./slot_space_info.txt"     #剪贴后的txt

key_words_dr = "slotSpaceInfo;"     #包括了planInit和Alg两个过程,init时为0

if __name__ == "__main__":
    print("start extract")

    argvstr = sys.argv
    if len(argvstr) < 2:
        print ("need input file name")
        # with open(argvstr[1], 'rb') as f:
        #     map = f.read()
        exit()
    else:        
        if argvstr[1][:9] == "mv_fusion":    #判断两个str的前11个字符是否完全一致
            print("The file is fusion log")
        else:
            print("The file is not fusion log")
            exit()

        with open(argvstr[1], 'rb') as f:
            # totalData = f.read()
            file_contents=f.readlines() 
            # f.closed


    fWrite_slot_space = open(slot_space_info_writePath,'w')
    

    for content in file_contents:   #content为bytes
         if key_words_dr.encode() in content:








    print("extract over")