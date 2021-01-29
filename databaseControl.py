import sqlite3

############################################
#######  CONNECTING TO DATABASE  ###########
############################################

database_filename = 'building101.db'

conn = sqlite3.connect(database_filename)

c = conn.cursor()

#c.execute("""INSERT INTO locations VALUES ('Main Reception', 25.365744908596728, 37.27408933229214,
# 0.0, 0.0, 0.4835940951036678, 0.8752923803968962);""")

c.execute("select * from locations")

locations = c.fetchall()

conn.commit()

print(locations)

conn.close()