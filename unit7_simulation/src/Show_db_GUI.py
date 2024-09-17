import mysql.connector
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from tkinter import simpledialog

# ฟังก์ชันสำหรับแสดงข้อมูลในตาราง record_tb
def show_records():
    # เชื่อมต่อกับฐานข้อมูล
    connection = mysql.connector.connect(
        host='localhost',
        user='agv_mysql',
        password='123qwe',
        database='AGV_db'
    )

    cursor = connection.cursor()

    # ลบข้อมูลเก่าทั้งหมดในตารางก่อนที่จะแสดงข้อมูลใหม่
    for row in record_table.get_children():
        record_table.delete(row)

    # ดึงข้อมูลจากฐานข้อมูล
    cursor.execute("SELECT * FROM record_tb")
    records = cursor.fetchall()

    # แสดงข้อมูลในตาราง
    for record in records:
        record_table.insert("", "end", values=record)

    connection.close()

# ฟังก์ชันสำหรับลบข้อมูลทั้งหมดในตาราง record_tb
def delete_all_records():
    confirm_code = simpledialog.askstring("Confirmation", "Enter confirmation code (1234):", parent=root)
    if confirm_code == '1234':
        # เชื่อมต่อกับฐานข้อมูล
        connection = mysql.connector.connect(
            host='localhost',
            user='agv_mysql',
            password='123qwe',
            database='AGV_db'
        )

        cursor = connection.cursor()

        # ลบข้อมูลทั้งหมดในตาราง
        cursor.execute("DELETE FROM record_tb")
        connection.commit()

        # ลบข้อมูลทั้งหมดในตารางแสดงผลบน GUI
        for row in record_table.get_children():
            record_table.delete(row)

        messagebox.showinfo("Success", "All records deleted successfully.")

        connection.close()
    else:
        messagebox.showerror("Error", "Confirmation code incorrect.")

# สร้างหน้าต่าง Tkinter
root = Tk()
root.title("Show Records")

# สร้าง Frame สำหรับเก็บตารางและ Scrollbar
frame = Frame(root)
frame.pack(padx=10, pady=10, fill=BOTH, expand=True)

# สร้าง Scrollbar แนวตั้ง
scrollbar = Scrollbar(frame, orient="vertical")

# สร้างตารางเพื่อแสดงข้อมูลใน record_tb
record_table = ttk.Treeview(frame, columns=("ID", "Date", "Time", "Start Point", "Finish Point", "Result", "Start Time", "Finish Time", "Duration", "Distance", "Battery_remain", "Battery_usage"), show="headings", yscrollcommand=scrollbar.set)
record_table.heading("ID", text="ID")
record_table.heading("Date", text="Date")
record_table.heading("Time", text="Time")
record_table.heading("Start Point", text="Start Point")
record_table.heading("Finish Point", text="Finish Point")
record_table.heading("Result", text="Result")
record_table.heading("Start Time", text="Start Time")
record_table.heading("Finish Time", text="Finish Time")
record_table.heading("Duration", text="Duration")
record_table.heading("Distance", text="Distance")
record_table.heading("Battery_remain", text="Battery_remain")
record_table.heading("Battery_usage", text="Battery_usage")

record_table.column("ID", width=50)
record_table.column("Date", width=100)
record_table.column("Time", width=80)
record_table.column("Start Point", width=100)
record_table.column("Finish Point", width=100)
record_table.column("Result", width=50)
record_table.column("Start Time", width=100)
record_table.column("Finish Time", width=100)
record_table.column("Duration", width=80)
record_table.column("Distance", width=80)
record_table.column("Battery_remain", width=130)
record_table.column("Battery_usage", width=130)

# กำหนด Scrollbar เพื่อควบคุมการเลื่อนข้อมูล
scrollbar.config(command=record_table.yview)
scrollbar.pack(side="right", fill="y")

record_table.pack(fill="both", expand=True)

# สร้างปุ่ม "Show" เพื่อแสดงข้อมูลในตาราง
Button(root, text="Show Records", command=show_records).pack(pady=10)

# สร้างปุ่ม "Delete All Records" เพื่อลบข้อมูลทั้งหมดในตาราง
Button(root, text="Delete All Records", command=delete_all_records).pack(pady=10)

# แสดงหน้าต่าง Tkinter
root.mainloop()
