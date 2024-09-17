import mysql.connector
from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import paho.mqtt.client as mqtt

# MQTT setup
host = 'localhost'
port = 1883
topic = '/ros_mqtt'
client = mqtt.Client()


# ฟังก์ชันสำหรับเพิ่มหรืออัปเดตข้อมูลในฐานข้อมูล
def add_or_update_goal():
    goal_point = goal_point_entry.get()
    goal_x = goal_x_entry.get()
    goal_y = goal_y_entry.get()
    goal_th = goal_th_entry.get()

    # เชื่อมต่อกับฐานข้อมูล
    connection = mysql.connector.connect(
        host='localhost',
        user='agv_mysql',
        password='123qwe',
        database='AGV_db'
    )

    cursor = connection.cursor()

    # ตรวจสอบว่า goal_point มีอยู่ในฐานข้อมูลหรือไม่
    cursor.execute("SELECT * FROM goal_tb WHERE goal_point = %s", (goal_point,))
    existing_goal = cursor.fetchone()

    if existing_goal:
        # อัปเดตข้อมูล
        update_query = "UPDATE goal_tb SET goal_x = %s, goal_y = %s, goal_th = %s WHERE goal_point = %s"
        cursor.execute(update_query, (goal_x, goal_y, goal_th, goal_point))
        connection.commit()
        messagebox.showinfo("Success", "Goal updated successfully")
    else:
        # เพิ่มข้อมูล
        insert_query = "INSERT INTO goal_tb (goal_point, goal_x, goal_y, goal_th) VALUES (%s, %s, %s, %s)"
        cursor.execute(insert_query, (goal_point, goal_x, goal_y, goal_th))
        connection.commit()
        messagebox.showinfo("Success", "Goal added successfully")

    connection.close()

# ฟังก์ชันสำหรับค้นหาและลบข้อมูลจากฐานข้อมูล
def delete_goal():
    goal_point = search_goal_entry.get()

    # เชื่อมต่อกับฐานข้อมูล
    connection = mysql.connector.connect(
        host='localhost',
        user='agv_mysql',
        password='123qwe',
        database='AGV_db'
    )

    cursor = connection.cursor()

    # ตรวจสอบว่า goal_point มีอยู่ในฐานข้อมูลหรือไม่
    cursor.execute("SELECT * FROM goal_tb WHERE goal_point = %s", (goal_point,))
    existing_goal = cursor.fetchone()

    if (existing_goal):
        # ลบข้อมูล
        delete_query = "DELETE FROM goal_tb WHERE goal_point = %s"
        cursor.execute(delete_query, (goal_point,))
        connection.commit()
        messagebox.showinfo("Success", "Goal deleted successfully")
    else:
        messagebox.showerror("Error", "Goal not found")

    connection.close()

# ฟังก์ชันสำหรับแสดงข้อมูลในตาราง
def show_goals():
    # เชื่อมต่อกับฐานข้อมูล
    connection = mysql.connector.connect(
        host='localhost',
        user='agv_mysql',
        password='123qwe',
        database='AGV_db'
    )

    cursor = connection.cursor()

    # ลบข้อมูลเก่าทั้งหมดในตารางก่อนที่จะแสดงข้อมูลใหม่
    for row in goal_table.get_children():
        goal_table.delete(row)

    # ดึงข้อมูลจากฐานข้อมูล
    cursor.execute("SELECT * FROM goal_tb")
    goals = cursor.fetchall()

    # แสดงข้อมูลในตาราง
    for goal in goals:
        goal_table.insert("", "end", values=goal)

    connection.close()

# ฟังก์ชันสำหรับส่งข้อความผ่าน MQTT
def send_mqtt_message():
    message = mqtt_message_entry.get()

    # เชื่อมต่อกับฐานข้อมูล
    connection = mysql.connector.connect(
        host='localhost',
        user='agv_mysql',
        password='123qwe',
        database='AGV_db'
    )

    cursor = connection.cursor()

    # ตรวจสอบว่า message มีอยู่ใน goal_point หรือไม่
    cursor.execute("SELECT * FROM goal_tb WHERE goal_point = %s", (message,))
    existing_goal = cursor.fetchone()

    connection.close()

    if existing_goal:
        client.connect(host, port, 60)
        client.publish(topic, message)
        client.disconnect(host, port)
        messagebox.showinfo("Success", "Message sent successfully")
    else:
        messagebox.showerror("Error", "Goal Point not found in database")



# MQTT setup
host = 'localhost'
port = 1883
topic = '/ros_mqtt'
client = mqtt.Client()
client.connect(host, port, 60)

# สร้างหน้าต่าง Tkinter
root = Tk()
root.title("Goal Management")

# สร้างช่องให้ผู้ใช้กรอกข้อมูล goal_point, goal_x, goal_y, goal_th
Label(root, text="Goal Point:").grid(row=0, column=0, padx=10, pady=5, sticky="w")
Label(root, text="Goal X:").grid(row=1, column=0, padx=10, pady=5, sticky="w")
Label(root, text="Goal Y:").grid(row=2, column=0, padx=10, pady=5, sticky="w")
Label(root, text="Goal Theta:").grid(row=3, column=0, padx=10, pady=5, sticky="w")

goal_point_entry = Entry(root)
goal_x_entry = Entry(root)
goal_y_entry = Entry(root)
goal_th_entry = Entry(root)

goal_point_entry.grid(row=0, column=1, padx=10, pady=5)
goal_x_entry.grid(row=1, column=1, padx=10, pady=5)
goal_y_entry.grid(row=2, column=1, padx=10, pady=5)
goal_th_entry.grid(row=3, column=1, padx=10, pady=5)

# สร้างปุ่ม "Add/Update" เพื่อเพิ่มหรืออัปเดตข้อมูล
Button(root, text="Add/Update", command=add_or_update_goal).grid(row=4, column=0, columnspan=2, pady=10, sticky="e")

# สร้างช่องให้ผู้ใช้กรอกข้อมูล goal_point เพื่อค้นหาและลบข้อมูล
Label(root, text="Delete Goal Point:").grid(row=5, column=0, padx=10, pady=5, sticky="w")
search_goal_entry = Entry(root)
search_goal_entry.grid(row=5, column=1, padx=10, pady=5)

# สร้างปุ่ม "Delete" เพื่อลบข้อมูล
Button(root, text="Delete", command=delete_goal).grid(row=6, column=0, columnspan=2, pady=10, sticky="e")

# สร้างตารางเพื่อแสดงข้อมูลใน goal_tb
goal_table = ttk.Treeview(root, columns=("Goal Point", "Goal X", "Goal Y", "Goal Theta"), show="headings")
goal_table.heading("Goal Point", text="Goal Point")
goal_table.heading("Goal X", text="Goal X")
goal_table.heading("Goal Y", text="Goal Y")
goal_table.heading("Goal Theta", text="Goal Theta")
goal_table.grid(row=7, column=0, columnspan=2, padx=10, pady=10)

# สร้างปุ่ม "Show" เพื่อแสดงข้อมูลในตาราง
Button(root, text="Show", command=show_goals).grid(row=8, column=0, columnspan=2, pady=10)

# สร้างช่องให้ผู้ใช้กรอกข้อความสำหรับส่งผ่าน MQTT
Label(root, text="MQTT Message:").grid(row=9, column=0, padx=10, pady=5, sticky="w")
mqtt_message_entry = Entry(root)
mqtt_message_entry.grid(row=9, column=1, padx=10, pady=5)

# สร้างปุ่ม "Send MQTT" เพื่อส่งข้อความผ่าน MQTT
Button(root, text="Send MQTT", command=send_mqtt_message).grid(row=10, column=0, columnspan=2, pady=10, sticky="e")

# แสดงหน้าต่าง Tkinter
root.mainloop()
