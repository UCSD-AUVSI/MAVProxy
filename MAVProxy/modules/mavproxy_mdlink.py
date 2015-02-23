"""
  MAVProxy mdlink

  uses lib/console.py for display
"""

import os, sys, math, time

from MAVProxy.modules.lib import wxconsole
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.mavproxy_map import mp_elevation
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import wxsettings
from MAVProxy.modules.lib.mp_menu import *

import mdlink_to_MissionDirector_connection


class MDlinkModule(mp_module.MPModule):
	def __init__(self, mpstate):
		super(MDlinkModule, self).__init__(mpstate, "mdlink", "GUI mdlink", public=True)
		self.in_air = False
		self.start_time = 0.0
		self.total_time = 0.0
		self.speed = 0
		self.max_link_num = 0
		mpstate.mdlink = wxconsole.MessageConsole(title='MDlink')
		
		#========================================================================
		self.MissionDirector_time_between_hbs = 1
		self.MissionDirector_time_of_last_hb = -100
		
		self.current_wp = 0
		self.wp_to_loiter_at = 3
		self.loiter_command_given = 0
		self.time_when_loiter_given = 0.0
		self.time_to_loiter_until_returning_home = 60.0
		self.return_home_command_given = 0
		#========================================================================
		
		# setup some default status information
		mpstate.mdlink.set_status('Mode', 'UNKNOWN', row=0, fg='blue')
		mpstate.mdlink.set_status('GPS', 'GPS: --', fg='red', row=0)
		mpstate.mdlink.set_status('Vcc', 'Vcc: --', fg='red', row=0)
		mpstate.mdlink.set_status('Radio', 'Radio: --', row=0)
		mpstate.mdlink.set_status('INS', 'INS', fg='grey', row=0)
		mpstate.mdlink.set_status('MAG', 'MAG', fg='grey', row=0)
		mpstate.mdlink.set_status('AS', 'AS', fg='grey', row=0)
		mpstate.mdlink.set_status('RNG', 'RNG', fg='grey', row=0)
		mpstate.mdlink.set_status('AHRS', 'AHRS', fg='grey', row=0)
		mpstate.mdlink.set_status('Heading', 'Hdg ---/---', row=2)
		mpstate.mdlink.set_status('Alt', 'Alt ---', row=2)
		mpstate.mdlink.set_status('AGL', 'AGL ---/---', row=2)
		mpstate.mdlink.set_status('AirSpeed', 'AirSpeed --', row=2)
		mpstate.mdlink.set_status('GPSSpeed', 'GPSSpeed --', row=2)
		mpstate.mdlink.set_status('Thr', 'Thr ---', row=2)
		mpstate.mdlink.set_status('Roll', 'Roll ---', row=2)
		mpstate.mdlink.set_status('Pitch', 'Pitch ---', row=2)
		mpstate.mdlink.set_status('Wind', 'Wind ---/---', row=2)
		mpstate.mdlink.set_status('WP', 'WP --', row=3)
		mpstate.mdlink.set_status('WPDist', 'Distance ---', row=3)
		mpstate.mdlink.set_status('WPBearing', 'Bearing ---', row=3)
		mpstate.mdlink.set_status('AltError', 'AltError --', row=3)
		mpstate.mdlink.set_status('AspdError', 'AspdError --', row=3)
		mpstate.mdlink.set_status('FlightTime', 'FlightTime --', row=3)
		mpstate.mdlink.set_status('ETR', 'ETR --', row=3)

		mpstate.mdlink.ElevationMap = mp_elevation.ElevationModel()

		# create the main menu
		self.menu = MPMenuTop([])
		self.add_menu(MPMenuSubMenu('MAVProxy',
									items=[MPMenuItem('Settings', 'Settings', 'menuSettings'),
										   MPMenuItem('Map', 'Load Map', '# module load map')]))

	def add_menu(self, menu):
		'''add a new menu'''
		self.menu.add(menu)
		self.mpstate.mdlink.set_menu(self.menu, self.menu_callback)

	def unload(self):
		'''unload module'''
		self.mpstate.mdlink.close()
		self.mpstate.mdlink = textconsole.SimpleConsole()

	def menu_callback(self, m):
		'''called on menu selection'''
		if m.returnkey.startswith('# '):
			cmd = m.returnkey[2:]
			if m.handler is not None:
				if m.handler_result is None:
					return
				cmd += m.handler_result
			self.mpstate.functions.process_stdin(cmd)
		if m.returnkey == 'menuSettings':
			wxsettings.WXSettings(self.settings)
	
#===========================================================================================================

	def idle_task(self):
		#checkup on mission (this is called a few hundred times per second)
		
		if (int(self.total_time) - int(self.MissionDirector_time_of_last_hb)) >= self.MissionDirector_time_between_hbs:
			mdlink_to_MissionDirector_connection.send(str(int(self.total_time)))
			self.MissionDirector_time_of_last_hb = self.total_time
		
		MissionDirector_msgs = mdlink_to_MissionDirector_connection.check_for_msg()
		if len(MissionDirector_msgs) > 0:
			for msg2 in MissionDirector_msgs:
				print "received message from MissionDirector: \"" + str(msg2) + "\""
				self.mpstate.functions.process_stdin(str(msg2))
		
		
		if self.loiter_command_given == 0:
			
			if self.current_wp > self.wp_to_loiter_at:
				self.loiter_command_given = 1
				self.time_when_loiter_given = self.total_time
				self.mdlink.writeln("waypoint 4 reached, switching to loiter mode (for 1 minute)")
				self.mdlink.writeln("time when starting to loiter: "+str(self.time_when_loiter_given))
				self.mpstate.functions.process_stdin("mode loiter")
		
		elif self.return_home_command_given == 0:
			
			if self.total_time > (self.time_when_loiter_given + self.time_to_loiter_until_returning_home):
				self.return_home_command_given = 1
				self.mdlink.writeln("loitered for 1 minute, returning home")
				self.mpstate.functions.process_stdin("mode rtl")
	
#===========================================================================================================
	
	def estimated_time_remaining(self, lat, lon, wpnum, speed):
		'''estimate time remaining in mission in seconds'''
		idx = wpnum
		if wpnum >= self.module('wp').wploader.count():
			return 0
		distance = 0
		done = set()
		while idx < self.module('wp').wploader.count():
			if idx in done:
				break
			done.add(idx)
			w = self.module('wp').wploader.wp(idx)
			if w.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
				idx = int(w.param1)
				continue
			idx += 1
			if (w.x != 0 or w.y != 0) and w.command in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
														mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
														mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
														mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
														mavutil.mavlink.MAV_CMD_NAV_LAND,
														mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]:
				distance += mp_util.gps_distance(lat, lon, w.x, w.y)
				lat = w.x
				lon = w.y
				if w.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
					break
		return distance / speed



	def mavlink_packet(self, msg):
		'''handle an incoming mavlink packet'''
		if not isinstance(self.mdlink, wxconsole.MessageConsole):
			return
		if not self.mdlink.is_alive():
			self.mpstate.mdlink = textconsole.SimpleConsole()
			return
		type = msg.get_type()

		master = self.master
		# add some status fields
		if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
			if type == "GPS_RAW":
				num_sats1 = master.field('GPS_STATUS', 'satellites_visible', 0)
			else:
				num_sats1 = msg.satellites_visible
			num_sats2 = master.field('GPS2_RAW', 'satellites_visible', -1)
			if num_sats2 == -1:
				sats_string = "%u" % num_sats1
			else:
				sats_string = "%u/%u" % (num_sats1, num_sats2)
			if ((msg.fix_type == 3 and master.mavlink10()) or
				(msg.fix_type == 2 and not master.mavlink10())):
				self.mdlink.set_status('GPS', 'GPS: OK (%s)' % sats_string, fg='green')
			else:
				self.mdlink.set_status('GPS', 'GPS: %u (%s)' % (msg.fix_type, sats_string), fg='red')
			if master.mavlink10():
				gps_heading = int(self.mpstate.status.msgs['GPS_RAW_INT'].cog * 0.01)
			else:
				gps_heading = self.mpstate.status.msgs['GPS_RAW'].hdg
			self.mdlink.set_status('Heading', 'Hdg %s/%u' % (master.field('VFR_HUD', 'heading', '-'), gps_heading))
		elif type == 'VFR_HUD':
			if master.mavlink10():
				alt = master.field('GPS_RAW_INT', 'alt', 0) / 1.0e3
			else:
				alt = master.field('GPS_RAW', 'alt', 0)
			if self.module('wp').wploader.count() > 0:
				wp = self.module('wp').wploader.wp(0)
				home_lat = wp.x
				home_lng = wp.y
			else:
				home_lat = master.field('HOME', 'lat') * 1.0e-7
				home_lng = master.field('HOME', 'lon') * 1.0e-7
			lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
			lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
			rel_alt = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
			agl_alt = None
			if self.settings.basealt != 0:
				agl_alt = self.mdlink.ElevationMap.GetElevation(lat, lng)
				if agl_alt is not None:
					agl_alt = self.settings.basealt - agl_alt
			else:
				agl_alt_home = self.mdlink.ElevationMap.GetElevation(home_lat, home_lng)
				if agl_alt_home is not None:
					agl_alt = self.mdlink.ElevationMap.GetElevation(lat, lng)
				if agl_alt is not None:
					agl_alt = agl_alt_home - agl_alt
			if agl_alt is not None:
				agl_alt += rel_alt
				vehicle_agl = master.field('TERRAIN_REPORT', 'current_height', None)
				if vehicle_agl is None:
					vehicle_agl = '---'
				else:
					vehicle_agl = int(vehicle_agl)
				self.mdlink.set_status('AGL', 'AGL %u/%s' % (agl_alt, vehicle_agl))
			self.mdlink.set_status('Alt', 'Alt %u' % rel_alt)
			self.mdlink.set_status('AirSpeed', 'AirSpeed %u' % msg.airspeed)
			self.mdlink.set_status('GPSSpeed', 'GPSSpeed %u' % msg.groundspeed)
			self.mdlink.set_status('Thr', 'Thr %u' % msg.throttle)
			t = time.localtime(msg._timestamp)
			if msg.groundspeed > 3 and not self.in_air:
				self.in_air = True
				self.start_time = time.mktime(t)
			elif msg.groundspeed > 3 and self.in_air:
				self.total_time = time.mktime(t) - self.start_time
				self.mdlink.set_status('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
			elif msg.groundspeed < 3 and self.in_air:
				self.in_air = False
				self.total_time = time.mktime(t) - self.start_time
				self.mdlink.set_status('FlightTime', 'FlightTime %u:%02u' % (int(self.total_time)/60, int(self.total_time)%60))
		elif type == 'ATTITUDE':
			self.mdlink.set_status('Roll', 'Roll %u' % math.degrees(msg.roll))
			self.mdlink.set_status('Pitch', 'Pitch %u' % math.degrees(msg.pitch))
		elif type in ['SYS_STATUS']:
			sensors = { 'AS'   : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
						'MAG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
						'INS'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
						'AHRS' : mavutil.mavlink.MAV_SYS_STATUS_AHRS,
						'TERR' : mavutil.mavlink.MAV_SYS_STATUS_TERRAIN,
						'RNG'  : mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION}
			for s in sensors.keys():
				bits = sensors[s]
				present = ((msg.onboard_control_sensors_enabled & bits) == bits)
				healthy = ((msg.onboard_control_sensors_health & bits) == bits)
				if not present:
					fg = 'grey'
				elif not healthy:
					fg = 'red'
				else:
					fg = 'green'
				# for terrain show yellow if still loading
				if s == 'TERR' and fg == 'green' and master.field('TERRAIN_REPORT', 'pending', 0) != 0:
					fg = 'yellow'
				self.mdlink.set_status(s, s, fg=fg)

		elif type == 'WIND':
			self.mdlink.set_status('Wind', 'Wind %u/%.2f' % (msg.direction, msg.speed))

		elif type == 'HWSTATUS':
			if msg.Vcc >= 4600 and msg.Vcc <= 5300:
				fg = 'green'
			else:
				fg = 'red'
			self.mdlink.set_status('Vcc', 'Vcc %.2f' % (msg.Vcc * 0.001), fg=fg)
		elif type == 'POWER_STATUS':
			if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_CHANGED:
				fg = 'red'
			else:
				fg = 'green'
			status = 'PWR:'
			if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_USB_CONNECTED:
				status += 'U'
			if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_BRICK_VALID:
				status += 'B'
			if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_SERVO_VALID:
				status += 'S'
			if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_PERIPH_OVERCURRENT:
				status += 'O1'
			if msg.flags & mavutil.mavlink.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT:
				status += 'O2'
			self.mdlink.set_status('PWR', status, fg=fg)
			self.mdlink.set_status('Srv', 'Srv %.2f' % (msg.Vservo*0.001), fg='green')
		elif type in ['RADIO', 'RADIO_STATUS']:
			if msg.rssi < msg.noise+10 or msg.remrssi < msg.remnoise+10:
				fg = 'red'
			else:
				fg = 'black'
			self.mdlink.set_status('Radio', 'Radio %u/%u %u/%u' % (msg.rssi, msg.noise, msg.remrssi, msg.remnoise), fg=fg)
		elif type == 'HEARTBEAT':
			self.mdlink.set_status('Mode', '%s' % master.flightmode, fg='blue')
			if self.max_link_num != len(self.mpstate.mav_master):
				for i in range(self.max_link_num):
					self.mdlink.set_status('Link%u'%(i+1), '', row=1)
				self.max_link_num = len(self.mpstate.mav_master)
			for m in self.mpstate.mav_master:
				linkdelay = (self.mpstate.status.highest_msec - m.highest_msec)*1.0e-3
				linkline = "Link %u " % (m.linknum+1)
				if m.linkerror:
					linkline += "down"
					fg = 'red'
				else:
					packets_rcvd_percentage = 100
					if (m.mav_loss != 0): #avoid divide-by-zero
						packets_rcvd_percentage = (1.0 - (float(m.mav_loss) / float(m.mav_count))) * 100.0

					linkline += "OK (%u pkts, %.2fs delay, %u lost) %u%%" % (m.mav_count, linkdelay, m.mav_loss, packets_rcvd_percentage)
					if linkdelay > 1:
						fg = 'orange'
					else:
						fg = 'darkgreen'
				self.mdlink.set_status('Link%u'%m.linknum, linkline, row=1, fg=fg)
		elif type in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
			self.mdlink.set_status('WP', 'WP %u' % msg.seq)
			self.current_wp = msg.seq
			lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
			lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
			if lat != 0 and lng != 0:
				airspeed = master.field('VFR_HUD', 'airspeed', 30)
				if abs(airspeed - self.speed) > 5:
					self.speed = airspeed
				else:
					self.speed = 0.98*self.speed + 0.02*airspeed
				self.speed = max(1, self.speed)
				time_remaining = int(self.estimated_time_remaining(lat, lng, msg.seq, self.speed))
				self.mdlink.set_status('ETR', 'ETR %u:%02u' % (time_remaining/60, time_remaining%60))

		elif type == 'NAV_CONTROLLER_OUTPUT':
			self.mdlink.set_status('WPDist', 'Distance %u' % msg.wp_dist)
			self.mdlink.set_status('WPBearing', 'Bearing %u' % msg.target_bearing)
			if msg.alt_error > 0:
				alt_error_sign = "L"
			else:
				alt_error_sign = "H"
			if msg.aspd_error > 0:
				aspd_error_sign = "L"
			else:
				aspd_error_sign = "H"
			self.mdlink.set_status('AltError', 'AltError %d%s' % (msg.alt_error, alt_error_sign))
			self.mdlink.set_status('AspdError', 'AspdError %.1f%s' % (msg.aspd_error*0.01, aspd_error_sign))

def init(mpstate):
	'''initialise module'''
	return MDlinkModule(mpstate)
