from tkinter import *
from tkinter import filedialog
import os

window = Tk()
window.title("面向应急车辆的信号抢占方法")
window.geometry("480x200")

# lbl = Label(window, text="Hello")
# lbl.grid(column=0, row=0)

# 起点和终点
start_label = Label(window, text="起     点:")
start_label.grid(column=3, row=0, columnspan=1)
_start = Entry(window, width=10)
_start.grid(column=4, row=0, columnspan=2)
_start.focus()

end_label = Label(window, text="终点:")
end_label.grid(column=6, row=0, columnspan=1)
_end = Entry(window, width=10)
_end.grid(column=7, row=0, columnspan=2)

# 交通规模
selected = IntVar()
radio_label = Label(window, text="交通规模:")
rad1 = Radiobutton(window, text="畅通", value=1, variable=selected)
rad2 = Radiobutton(window, text="较为拥堵", value=2, variable=selected)
rad3 = Radiobutton(window, text="中度拥堵", value=3, variable=selected)
rad4 = Radiobutton(window, text="严重拥堵", value=4, variable=selected)

radio_label.grid(column=3, row=2, columnspan=1)
rad1.grid(column=4, row=2, columnspan=1)
rad2.grid(column=5, row=2, columnspan=1)
rad3.grid(column=6, row=2, columnspan=1)
rad4.grid(column=7, row=2, columnspan=1)


file_label_path1 = Label(window, text="文件路径:")
file_label_path2 = Label(window, text="")
# 地图文件

global file

def file_clicked():
    global file
    file = filedialog.askopenfilename(
        filetypes=(("Text files", "*.xml"), ("all files", "*.*")),
        initialdir=os.path.dirname(__file__)
    )
    file_label_path2.configure(text=file)

file_label = Label(window, text="地     图:")
file_btn = Button(window, text="选择", command=file_clicked)

file_label.grid(column=3, row=3)
file_btn.grid(column=4, row=3)
file_label_path1.grid(column=3, row=4)
file_label_path2.grid(column=4, row=4, columnspan=5)

def clicked():
    _from_ = _start.get()
    _end_ = _end.get()
    _scale_ = selected.get()
    _path_ = file
    print(_from_, _end_, _scale_, _path_)


btn = Button(window, text="确认", command=clicked)
btn.grid(column=3, row=5)

window.mainloop()
