# for Python3
import tkinter as tk

class USBSelect:
    def __init__(self):
        self.HEIGHT = 700
        self.WIDTH = 800    
        root = tk.Tk()
        root.width = self.WIDTH
        root.height = self.HEIGHT
        self.dialogroot = root
        self.strDialogResult = ""    
        self.canvas = tk.Canvas(root, height=self.HEIGHT, width=self.WIDTH)
        self.canvas.pack()    
        frame = tk.Frame(root, bg='#42c2f4')
        frame.place(relx=0.5, rely=0.02, relwidth=0.96, relheight=0.95, anchor='n')  
        # Here is the button call to the InputBox() function
        buttonInputBox = tk.Button(frame, text="Input Box", bg='#cccccc', font=60, 
            command=lambda: self.InputBox())   # open the window   
        buttonInputBox.place(relx=0.05, rely=0.1, relwidth=0.90, relheight=0.8)    
        
        root.mainloop()   # execute program

    def InputBox(self):        
        dialog = tk.Toplevel(self.dialogroot)
        dialog.width = 600
        dialog.height = 100

        frame = tk.Frame(dialog,  bg='#42c2f4', bd=5)
        frame.place(relwidth=1, relheight=1)

        entry = tk.Entry(frame, font=40)
        entry.place(relwidth=0.65, rely=0.02, relheight=0.96)
        entry.focus_set()

        submit = tk.Button(frame, text='OK', font=16, command=lambda: self.DialogResult(entry.get()))
        submit.place(relx=0.7, rely=0.02, relheight=0.96, relwidth=0.3)

        root_name = self.dialogroot.winfo_pathname(self.dialogroot.winfo_id())
        dialog_name = dialog.winfo_pathname(dialog.winfo_id())

        # These two lines show a modal dialog
        self.dialogroot.tk.eval('tk::PlaceWindow {0} widget {1}'.format(dialog_name, root_name))
        #self.dialogroot.wm_withdraw()   # hides main host window
        self.dialogroot.mainloop()

        #This line destroys the modal dialog after the user exits/accepts it
        dialog.destroy()

        #Print and return the inputbox result
        print(self.strDialogResult)
        return self.strDialogResult

    def DialogResult(self, result):
        self.strDialogResult = result
        #This line quits from the MODAL STATE but doesn't close or destroy the modal dialog
        self.dialogroot.quit()


# Launch ...
if __name__ == '__main__':
    go = USBSelect()   # launch app