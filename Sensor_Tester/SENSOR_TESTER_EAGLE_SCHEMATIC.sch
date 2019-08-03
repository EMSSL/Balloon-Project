<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.4.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="16" fill="1" visible="no" active="no"/>
<layer number="3" name="Route3" color="17" fill="1" visible="no" active="no"/>
<layer number="4" name="Route4" color="18" fill="1" visible="no" active="no"/>
<layer number="5" name="Route5" color="19" fill="1" visible="no" active="no"/>
<layer number="6" name="Route6" color="25" fill="1" visible="no" active="no"/>
<layer number="7" name="Route7" color="26" fill="1" visible="no" active="no"/>
<layer number="8" name="Route8" color="27" fill="1" visible="no" active="no"/>
<layer number="9" name="Route9" color="28" fill="1" visible="no" active="no"/>
<layer number="10" name="Route10" color="29" fill="1" visible="no" active="no"/>
<layer number="11" name="Route11" color="30" fill="1" visible="no" active="no"/>
<layer number="12" name="Route12" color="20" fill="1" visible="no" active="no"/>
<layer number="13" name="Route13" color="21" fill="1" visible="no" active="no"/>
<layer number="14" name="Route14" color="22" fill="1" visible="no" active="no"/>
<layer number="15" name="Route15" color="23" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="7" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="Adafruit_Arduino">
<packages>
<package name="ADAFRUIT_GPS">
<wire x1="-12.7" y1="17.018" x2="12.7" y2="17.018" width="0.127" layer="21"/>
<wire x1="12.7" y1="17.018" x2="12.7" y2="-17.018" width="0.127" layer="21"/>
<wire x1="12.7" y1="-17.018" x2="-12.7" y2="-17.018" width="0.127" layer="21"/>
<wire x1="-12.7" y1="-17.018" x2="-12.7" y2="17.018" width="0.127" layer="21"/>
<pad name="3.3V" x="-10.16" y="-14.986" drill="1"/>
<pad name="EN" x="-7.62" y="-14.986" drill="1"/>
<pad name="VBAT" x="-5.08" y="-14.986" drill="1"/>
<pad name="FIX" x="-2.54" y="-14.986" drill="1"/>
<pad name="TX" x="0" y="-14.986" drill="1"/>
<pad name="RX" x="2.54" y="-14.986" drill="1"/>
<pad name="GND" x="5.08" y="-14.986" drill="1"/>
<pad name="VIN" x="7.62" y="-14.986" drill="1"/>
<pad name="PPS" x="10.16" y="-14.986" drill="1"/>
<text x="-1.778" y="-0.254" size="1.27" layer="21">GPS</text>
</package>
<package name="ADAFRUIT_SD_RW">
<wire x1="-12.7" y1="15.875" x2="12.7" y2="15.875" width="0.127" layer="21"/>
<wire x1="12.7" y1="15.875" x2="12.7" y2="-15.875" width="0.127" layer="21"/>
<wire x1="12.7" y1="-15.875" x2="-12.7" y2="-15.875" width="0.127" layer="21"/>
<wire x1="-12.7" y1="-15.875" x2="-12.7" y2="15.875" width="0.127" layer="21"/>
<pad name="5V" x="-8.89" y="-14.224" drill="1"/>
<pad name="3V" x="-6.35" y="-14.224" drill="1"/>
<pad name="GND" x="-3.81" y="-14.224" drill="1"/>
<pad name="CLK" x="-1.27" y="-14.224" drill="1"/>
<pad name="MISO" x="1.27" y="-14.224" drill="1"/>
<pad name="MOSI" x="3.81" y="-14.224" drill="1"/>
<pad name="CS" x="6.35" y="-14.224" drill="1"/>
<pad name="CD" x="8.89" y="-14.224" drill="1"/>
<text x="-3.048" y="1.778" size="1.27" layer="21">SD_RW</text>
</package>
<package name="ADS1115">
<wire x1="-13.97" y1="-8.636" x2="-13.97" y2="8.636" width="0.127" layer="21"/>
<wire x1="-13.97" y1="8.636" x2="13.97" y2="8.636" width="0.127" layer="21"/>
<wire x1="13.97" y1="8.636" x2="13.97" y2="-8.636" width="0.127" layer="21"/>
<wire x1="13.97" y1="-8.636" x2="-13.97" y2="-8.636" width="0.127" layer="21"/>
<pad name="VDD" x="-11.43" y="-6.604" drill="1"/>
<pad name="GND" x="-8.89" y="-6.604" drill="1"/>
<pad name="SCL" x="-6.35" y="-6.604" drill="1"/>
<pad name="SDA" x="-3.81" y="-6.604" drill="1"/>
<pad name="ADDR" x="-1.27" y="-6.604" drill="1"/>
<pad name="ALRT" x="1.27" y="-6.604" drill="1"/>
<pad name="A0" x="3.81" y="-6.604" drill="1"/>
<pad name="A1" x="6.35" y="-6.604" drill="1"/>
<pad name="A2" x="8.89" y="-6.604" drill="1"/>
<pad name="A3" x="11.43" y="-6.604" drill="1"/>
<text x="-2.54" y="1.016" size="1.778" layer="21">ADC</text>
</package>
<package name="BME280">
<wire x1="-8.89" y1="-9.525" x2="-8.89" y2="9.525" width="0.127" layer="21"/>
<wire x1="-8.89" y1="9.525" x2="8.89" y2="9.525" width="0.127" layer="21"/>
<wire x1="8.89" y1="9.525" x2="8.89" y2="-9.525" width="0.127" layer="21"/>
<wire x1="8.89" y1="-9.525" x2="-8.89" y2="-9.525" width="0.127" layer="21"/>
<pad name="VIN" x="-7.62" y="-6.9596" drill="1"/>
<pad name="3VO" x="-5.08" y="-6.9596" drill="1"/>
<pad name="GND" x="-2.54" y="-6.9596" drill="1"/>
<pad name="SCK" x="0" y="-6.9596" drill="1"/>
<pad name="MISO" x="2.54" y="-6.9596" drill="1"/>
<pad name="MOSI" x="5.08" y="-6.9596" drill="1"/>
<pad name="CS" x="7.62" y="-6.9596" drill="1"/>
<text x="-1.27" y="1.778" size="1.27" layer="21">ALT</text>
</package>
<package name="BNO055">
<wire x1="-13.335" y1="-10.16" x2="-13.335" y2="10.16" width="0.127" layer="21"/>
<wire x1="-13.335" y1="10.16" x2="13.335" y2="10.16" width="0.127" layer="21"/>
<wire x1="13.335" y1="10.16" x2="13.335" y2="-10.16" width="0.127" layer="21"/>
<wire x1="13.335" y1="-10.16" x2="-13.335" y2="-10.16" width="0.127" layer="21"/>
<pad name="PS0" x="-3.81" y="8.89" drill="1"/>
<pad name="PS1" x="-1.27" y="8.89" drill="1"/>
<pad name="INT" x="1.27" y="8.89" drill="1"/>
<pad name="ADDR" x="3.81" y="8.89" drill="1"/>
<pad name="VIN" x="-6.35" y="-8.89" drill="1"/>
<pad name="3V0" x="-3.81" y="-8.89" drill="1"/>
<pad name="GND" x="-1.27" y="-8.89" drill="1"/>
<pad name="SDA" x="1.27" y="-8.89" drill="1"/>
<pad name="SCL" x="3.81" y="-8.89" drill="1"/>
<pad name="RST" x="6.35" y="-8.89" drill="1"/>
<text x="-1.524" y="1.016" size="1.27" layer="21">IMU</text>
</package>
</packages>
<symbols>
<symbol name="ADAFRUIT_GPS">
<wire x1="-12.7" y1="-17.018" x2="12.7" y2="-17.018" width="0.254" layer="94"/>
<wire x1="12.7" y1="-17.018" x2="12.7" y2="17.018" width="0.254" layer="94"/>
<wire x1="12.7" y1="17.018" x2="-12.7" y2="17.018" width="0.254" layer="94"/>
<wire x1="-12.7" y1="17.018" x2="-12.7" y2="-17.018" width="0.254" layer="94"/>
<pin name="3.3V" x="-10.16" y="-22.098" length="middle" rot="R90"/>
<pin name="EN" x="-7.62" y="-22.098" length="middle" rot="R90"/>
<pin name="VBAT" x="-5.08" y="-22.098" length="middle" rot="R90"/>
<pin name="FIX" x="-2.54" y="-22.098" length="middle" rot="R90"/>
<pin name="TX" x="0" y="-22.098" length="middle" rot="R90"/>
<pin name="RX" x="2.54" y="-22.098" length="middle" rot="R90"/>
<pin name="GND" x="5.08" y="-22.098" length="middle" rot="R90"/>
<pin name="VIN" x="7.62" y="-22.098" length="middle" rot="R90"/>
<pin name="PPS" x="10.16" y="-22.098" length="middle" rot="R90"/>
<text x="-7.62" y="1.27" size="5.08" layer="94">GPS</text>
</symbol>
<symbol name="ADAFRUIT_SD">
<wire x1="-12.7" y1="-15.875" x2="-12.7" y2="15.875" width="0.254" layer="94"/>
<wire x1="-12.7" y1="15.875" x2="12.7" y2="15.875" width="0.254" layer="94"/>
<wire x1="12.7" y1="15.875" x2="12.7" y2="-15.875" width="0.254" layer="94"/>
<wire x1="12.7" y1="-15.875" x2="-12.7" y2="-15.875" width="0.254" layer="94"/>
<text x="-6.096" y="5.334" size="2.54" layer="94">SD_RW</text>
<pin name="5V" x="-8.89" y="-21.082" length="middle" rot="R90"/>
<pin name="3V" x="-6.35" y="-21.082" length="middle" rot="R90"/>
<pin name="GND" x="-3.81" y="-21.082" length="middle" rot="R90"/>
<pin name="CLK" x="-1.27" y="-21.082" length="middle" rot="R90"/>
<pin name="MISO" x="1.27" y="-21.082" length="middle" rot="R90"/>
<pin name="MOSI" x="3.81" y="-21.082" length="middle" rot="R90"/>
<pin name="CS" x="6.35" y="-21.082" length="middle" rot="R90"/>
<pin name="CD" x="8.89" y="-21.082" length="middle" rot="R90"/>
</symbol>
<symbol name="ADS1115">
<wire x1="-13.97" y1="-8.636" x2="-13.97" y2="8.636" width="0.254" layer="94"/>
<wire x1="-13.97" y1="8.636" x2="13.97" y2="8.636" width="0.254" layer="94"/>
<wire x1="13.97" y1="8.636" x2="13.97" y2="-8.636" width="0.254" layer="94"/>
<wire x1="13.97" y1="-8.636" x2="-13.97" y2="-8.636" width="0.254" layer="94"/>
<pin name="VDD" x="-11.43" y="-13.716" length="middle" rot="R90"/>
<pin name="GND" x="-8.89" y="-13.716" length="middle" rot="R90"/>
<pin name="SCL" x="-6.35" y="-13.716" length="middle" rot="R90"/>
<pin name="SDA" x="-3.81" y="-13.716" length="middle" rot="R90"/>
<pin name="ADDR" x="-1.27" y="-13.716" length="middle" rot="R90"/>
<pin name="ALRT" x="1.27" y="-13.716" length="middle" rot="R90"/>
<pin name="A0" x="3.81" y="-13.716" length="middle" rot="R90"/>
<pin name="A1" x="6.35" y="-13.716" length="middle" rot="R90"/>
<pin name="A2" x="8.89" y="-13.716" length="middle" rot="R90"/>
<pin name="A3" x="11.43" y="-13.716" length="middle" rot="R90"/>
<text x="-2.54" y="2.54" size="2.54" layer="94">ADC</text>
</symbol>
<symbol name="BME280">
<wire x1="-8.89" y1="-9.525" x2="-8.89" y2="9.525" width="0.254" layer="94"/>
<wire x1="-8.89" y1="9.525" x2="8.89" y2="9.525" width="0.254" layer="94"/>
<wire x1="8.89" y1="9.525" x2="8.89" y2="-9.525" width="0.254" layer="94"/>
<wire x1="8.89" y1="-9.525" x2="-8.89" y2="-9.525" width="0.254" layer="94"/>
<pin name="VIN" x="-7.62" y="-14.732" length="middle" rot="R90"/>
<pin name="3V0" x="-5.08" y="-14.732" length="middle" rot="R90"/>
<pin name="GND" x="-2.54" y="-14.732" length="middle" rot="R90"/>
<pin name="SCK" x="0" y="-14.732" length="middle" rot="R90"/>
<pin name="MISO" x="2.54" y="-14.732" length="middle" rot="R90"/>
<pin name="MOSI" x="5.08" y="-14.732" length="middle" rot="R90"/>
<pin name="CS" x="7.62" y="-14.732" length="middle" rot="R90"/>
<text x="0" y="2.54" size="1.27" layer="94">ALT</text>
</symbol>
<symbol name="BNO055">
<wire x1="-13.335" y1="-10.16" x2="-13.335" y2="10.16" width="0.254" layer="94"/>
<wire x1="-13.335" y1="10.16" x2="13.335" y2="10.16" width="0.254" layer="94"/>
<wire x1="13.335" y1="10.16" x2="13.335" y2="-10.16" width="0.254" layer="94"/>
<wire x1="13.335" y1="-10.16" x2="-13.335" y2="-10.16" width="0.254" layer="94"/>
<pin name="PS0" x="-3.81" y="15.24" length="middle" rot="R270"/>
<pin name="PS1" x="-1.27" y="15.24" length="middle" rot="R270"/>
<pin name="INT" x="1.27" y="15.24" length="middle" rot="R270"/>
<pin name="ADDR" x="3.81" y="15.24" length="middle" rot="R270"/>
<pin name="VIN" x="-6.35" y="-15.24" length="middle" rot="R90"/>
<pin name="3V0" x="-3.81" y="-15.24" length="middle" rot="R90"/>
<pin name="GND" x="-1.27" y="-15.24" length="middle" rot="R90"/>
<pin name="SDA" x="1.27" y="-15.24" length="middle" rot="R90"/>
<pin name="SCL" x="3.81" y="-15.24" length="middle" rot="R90"/>
<pin name="RST" x="6.35" y="-15.24" length="middle" rot="R90"/>
<text x="-5.08" y="0" size="1.27" layer="94">IMU</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ADAFRUIT_GPS">
<gates>
<gate name="G$1" symbol="ADAFRUIT_GPS" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ADAFRUIT_GPS">
<connects>
<connect gate="G$1" pin="3.3V" pad="3.3V"/>
<connect gate="G$1" pin="EN" pad="EN"/>
<connect gate="G$1" pin="FIX" pad="FIX"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="PPS" pad="PPS"/>
<connect gate="G$1" pin="RX" pad="RX"/>
<connect gate="G$1" pin="TX" pad="TX"/>
<connect gate="G$1" pin="VBAT" pad="VBAT"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="ADAFRUI_SD_RW">
<gates>
<gate name="G$1" symbol="ADAFRUIT_SD" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ADAFRUIT_SD_RW">
<connects>
<connect gate="G$1" pin="3V" pad="3V"/>
<connect gate="G$1" pin="5V" pad="5V"/>
<connect gate="G$1" pin="CD" pad="CD"/>
<connect gate="G$1" pin="CLK" pad="CLK"/>
<connect gate="G$1" pin="CS" pad="CS"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="MISO" pad="MISO"/>
<connect gate="G$1" pin="MOSI" pad="MOSI"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="ADS1115">
<gates>
<gate name="G$1" symbol="ADS1115" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ADS1115">
<connects>
<connect gate="G$1" pin="A0" pad="A0"/>
<connect gate="G$1" pin="A1" pad="A1"/>
<connect gate="G$1" pin="A2" pad="A2"/>
<connect gate="G$1" pin="A3" pad="A3"/>
<connect gate="G$1" pin="ADDR" pad="ADDR"/>
<connect gate="G$1" pin="ALRT" pad="ALRT"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="SCL" pad="SCL"/>
<connect gate="G$1" pin="SDA" pad="SDA"/>
<connect gate="G$1" pin="VDD" pad="VDD"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="BME280">
<gates>
<gate name="G$1" symbol="BME280" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BME280">
<connects>
<connect gate="G$1" pin="3V0" pad="3VO"/>
<connect gate="G$1" pin="CS" pad="CS"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="MISO" pad="MISO"/>
<connect gate="G$1" pin="MOSI" pad="MOSI"/>
<connect gate="G$1" pin="SCK" pad="SCK"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="BNO055">
<gates>
<gate name="G$1" symbol="BNO055" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BNO055">
<connects>
<connect gate="G$1" pin="3V0" pad="3V0"/>
<connect gate="G$1" pin="ADDR" pad="ADDR"/>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="INT" pad="INT"/>
<connect gate="G$1" pin="PS0" pad="PS0"/>
<connect gate="G$1" pin="PS1" pad="PS1"/>
<connect gate="G$1" pin="RST" pad="RST"/>
<connect gate="G$1" pin="SCL" pad="SCL"/>
<connect gate="G$1" pin="SDA" pad="SDA"/>
<connect gate="G$1" pin="VIN" pad="VIN"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="pinhead" urn="urn:adsk.eagle:library:325">
<description>&lt;b&gt;Pin Header Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="1X10" urn="urn:adsk.eagle:footprint:22264/1" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="7.62" y1="0.635" x2="8.255" y2="1.27" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.27" x2="9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="1.27" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0.635" x2="10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-0.635" x2="9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="9.525" y1="-1.27" x2="8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-1.27" x2="7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="4.445" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-0.635" x2="4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="1.27" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0.635" x2="7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-0.635" x2="6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-1.27" x2="5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.715" y1="-1.27" x2="5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.27" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-6.985" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="1.27" x2="-5.715" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-5.08" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-5.715" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.715" y1="-1.27" x2="-6.985" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-1.27" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.27" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="-1.27" x2="-4.445" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="1.27" x2="-10.795" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="1.27" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-0.635" x2="-10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-9.525" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="1.27" x2="-8.255" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.27" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-7.62" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-0.635" x2="-8.255" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-1.27" x2="-9.525" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-9.525" y1="-1.27" x2="-10.16" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="0.635" x2="-12.7" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-12.065" y1="1.27" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="-0.635" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-10.795" y1="-1.27" x2="-12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="10.795" y1="1.27" x2="12.065" y2="1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="1.27" x2="12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="0.635" x2="12.7" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-0.635" x2="12.065" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="10.795" y1="1.27" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-0.635" x2="10.795" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="12.065" y1="-1.27" x2="10.795" y2="-1.27" width="0.1524" layer="21"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="3.81" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="6.35" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="10" x="11.43" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-12.7762" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-12.7" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="8.636" y1="-0.254" x2="9.144" y2="0.254" layer="51"/>
<rectangle x1="6.096" y1="-0.254" x2="6.604" y2="0.254" layer="51"/>
<rectangle x1="3.556" y1="-0.254" x2="4.064" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="-4.064" y1="-0.254" x2="-3.556" y2="0.254" layer="51"/>
<rectangle x1="-6.604" y1="-0.254" x2="-6.096" y2="0.254" layer="51"/>
<rectangle x1="-9.144" y1="-0.254" x2="-8.636" y2="0.254" layer="51"/>
<rectangle x1="-11.684" y1="-0.254" x2="-11.176" y2="0.254" layer="51"/>
<rectangle x1="11.176" y1="-0.254" x2="11.684" y2="0.254" layer="51"/>
</package>
<package name="1X10/90" urn="urn:adsk.eagle:footprint:22265/1" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-12.7" y1="-1.905" x2="-10.16" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-1.905" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="0.635" x2="-12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="0.635" x2="-12.7" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="6.985" x2="-11.43" y2="1.27" width="0.762" layer="21"/>
<wire x1="-10.16" y1="-1.905" x2="-7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-1.905" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0.635" x2="-10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="6.985" x2="-8.89" y2="1.27" width="0.762" layer="21"/>
<wire x1="-7.62" y1="-1.905" x2="-5.08" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.635" x2="-7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="6.985" x2="-6.35" y2="1.27" width="0.762" layer="21"/>
<wire x1="-5.08" y1="-1.905" x2="-2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="6.985" x2="-3.81" y2="1.27" width="0.762" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.905" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="6.985" x2="-1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="0" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="6.985" x2="1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="5.08" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.905" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0.635" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="6.985" x2="3.81" y2="1.27" width="0.762" layer="21"/>
<wire x1="5.08" y1="-1.905" x2="7.62" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-1.905" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0.635" x2="5.08" y2="0.635" width="0.1524" layer="21"/>
<wire x1="6.35" y1="6.985" x2="6.35" y2="1.27" width="0.762" layer="21"/>
<wire x1="7.62" y1="-1.905" x2="10.16" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-1.905" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="10.16" y1="0.635" x2="7.62" y2="0.635" width="0.1524" layer="21"/>
<wire x1="8.89" y1="6.985" x2="8.89" y2="1.27" width="0.762" layer="21"/>
<wire x1="10.16" y1="-1.905" x2="12.7" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-1.905" x2="12.7" y2="0.635" width="0.1524" layer="21"/>
<wire x1="12.7" y1="0.635" x2="10.16" y2="0.635" width="0.1524" layer="21"/>
<wire x1="11.43" y1="6.985" x2="11.43" y2="1.27" width="0.762" layer="21"/>
<pad name="1" x="-11.43" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="-8.89" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="-6.35" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="4" x="-3.81" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="5" x="-1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="6" x="1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="7" x="3.81" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="8" x="6.35" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="10" x="11.43" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<text x="-13.335" y="-3.81" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="14.605" y="-4.445" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-11.811" y1="0.635" x2="-11.049" y2="1.143" layer="21"/>
<rectangle x1="-9.271" y1="0.635" x2="-8.509" y2="1.143" layer="21"/>
<rectangle x1="-6.731" y1="0.635" x2="-5.969" y2="1.143" layer="21"/>
<rectangle x1="-4.191" y1="0.635" x2="-3.429" y2="1.143" layer="21"/>
<rectangle x1="-1.651" y1="0.635" x2="-0.889" y2="1.143" layer="21"/>
<rectangle x1="0.889" y1="0.635" x2="1.651" y2="1.143" layer="21"/>
<rectangle x1="3.429" y1="0.635" x2="4.191" y2="1.143" layer="21"/>
<rectangle x1="5.969" y1="0.635" x2="6.731" y2="1.143" layer="21"/>
<rectangle x1="8.509" y1="0.635" x2="9.271" y2="1.143" layer="21"/>
<rectangle x1="11.049" y1="0.635" x2="11.811" y2="1.143" layer="21"/>
<rectangle x1="-11.811" y1="-2.921" x2="-11.049" y2="-1.905" layer="21"/>
<rectangle x1="-9.271" y1="-2.921" x2="-8.509" y2="-1.905" layer="21"/>
<rectangle x1="-6.731" y1="-2.921" x2="-5.969" y2="-1.905" layer="21"/>
<rectangle x1="-4.191" y1="-2.921" x2="-3.429" y2="-1.905" layer="21"/>
<rectangle x1="-1.651" y1="-2.921" x2="-0.889" y2="-1.905" layer="21"/>
<rectangle x1="0.889" y1="-2.921" x2="1.651" y2="-1.905" layer="21"/>
<rectangle x1="3.429" y1="-2.921" x2="4.191" y2="-1.905" layer="21"/>
<rectangle x1="5.969" y1="-2.921" x2="6.731" y2="-1.905" layer="21"/>
<rectangle x1="8.509" y1="-2.921" x2="9.271" y2="-1.905" layer="21"/>
<rectangle x1="11.049" y1="-2.921" x2="11.811" y2="-1.905" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="1X10" urn="urn:adsk.eagle:package:22406/2" type="model" library_version="3">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="1X10"/>
</packageinstances>
</package3d>
<package3d name="1X10/90" urn="urn:adsk.eagle:package:22408/2" type="model" library_version="3">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="1X10/90"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="PINHD10" urn="urn:adsk.eagle:symbol:22263/1" library_version="3">
<wire x1="-6.35" y1="-15.24" x2="1.27" y2="-15.24" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-15.24" x2="1.27" y2="12.7" width="0.4064" layer="94"/>
<wire x1="1.27" y1="12.7" x2="-6.35" y2="12.7" width="0.4064" layer="94"/>
<wire x1="-6.35" y1="12.7" x2="-6.35" y2="-15.24" width="0.4064" layer="94"/>
<text x="-6.35" y="13.335" size="1.778" layer="95">&gt;NAME</text>
<text x="-6.35" y="-17.78" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="10.16" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="2" x="-2.54" y="7.62" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="3" x="-2.54" y="5.08" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="4" x="-2.54" y="2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="5" x="-2.54" y="0" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="6" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="7" x="-2.54" y="-5.08" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="8" x="-2.54" y="-7.62" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="9" x="-2.54" y="-10.16" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="10" x="-2.54" y="-12.7" visible="pad" length="short" direction="pas" function="dot"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="PINHD-1X10" urn="urn:adsk.eagle:component:22498/4" prefix="JP" uservalue="yes" library_version="3">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="PINHD10" x="0" y="0"/>
</gates>
<devices>
<device name="" package="1X10">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="10" pad="10"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
<connect gate="A" pin="7" pad="7"/>
<connect gate="A" pin="8" pad="8"/>
<connect gate="A" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22406/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="/90" package="1X10/90">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="10" pad="10"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
<connect gate="A" pin="7" pad="7"/>
<connect gate="A" pin="8" pad="8"/>
<connect gate="A" pin="9" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:22408/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="Adafruit_Arduino" deviceset="ADAFRUIT_GPS" device=""/>
<part name="U$2" library="Adafruit_Arduino" deviceset="ADAFRUI_SD_RW" device=""/>
<part name="U$3" library="Adafruit_Arduino" deviceset="ADS1115" device=""/>
<part name="U$4" library="Adafruit_Arduino" deviceset="BME280" device=""/>
<part name="U$5" library="Adafruit_Arduino" deviceset="BNO055" device=""/>
<part name="MCU_PINS" library="pinhead" library_urn="urn:adsk.eagle:library:325" deviceset="PINHD-1X10" device="/90" package3d_urn="urn:adsk.eagle:package:22408/2"/>
</parts>
<sheets>
<sheet>
<plain>
<text x="44.45" y="106.68" size="1.778" layer="95">ADAFRUIT ULTIMATE 
GPS BREAKOUT</text>
<text x="93.98" y="106.68" size="1.778" layer="95">ADAFRUIT MICROSD
READ/WRITE</text>
<text x="139.7" y="92.71" size="1.778" layer="95">ADAFRUIT ADS1115</text>
<text x="180.34" y="92.71" size="1.778" layer="95">ADAFRUIT BME280
(MOSI = DI)</text>
<text x="215.9" y="102.87" size="1.778" layer="95">ADAFRUIT BNO055
</text>
<text x="60.96" y="132.08" size="1.778" layer="95">NOTE, ANY COMBO OF THE SENSOR IS OK, AS LONG AS CODE COMPATIBILITY SECTION OF SKETCH IS PROPERLY CONFIGURED</text>
<wire x1="0" y1="0" x2="0" y2="138.43" width="0.1524" layer="95"/>
<wire x1="0" y1="138.43" x2="256.54" y2="138.43" width="0.1524" layer="95"/>
<wire x1="256.54" y1="138.43" x2="256.54" y2="0" width="0.1524" layer="95"/>
<wire x1="256.54" y1="0" x2="0" y2="0" width="0.1524" layer="95"/>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="57.15" y="85.09" smashed="yes"/>
<instance part="U$2" gate="G$1" x="105.41" y="85.09" smashed="yes"/>
<instance part="U$3" gate="G$1" x="151.13" y="78.74" smashed="yes"/>
<instance part="U$4" gate="G$1" x="190.5" y="80.01" smashed="yes"/>
<instance part="U$5" gate="G$1" x="229.87" y="81.28" smashed="yes"/>
<instance part="MCU_PINS" gate="A" x="17.78" y="26.67" smashed="yes" rot="R180">
<attribute name="NAME" x="24.13" y="13.335" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="24.13" y="44.45" size="1.778" layer="96" rot="R180"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="PWR" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="10"/>
<pinref part="U$1" gate="G$1" pin="VIN"/>
<wire x1="20.32" y1="39.37" x2="64.77" y2="39.37" width="0.1524" layer="91"/>
<wire x1="64.77" y1="39.37" x2="64.77" y2="62.992" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="5V"/>
<wire x1="96.52" y1="64.008" x2="96.52" y2="39.37" width="0.1524" layer="91"/>
<wire x1="96.52" y1="39.37" x2="64.77" y2="39.37" width="0.1524" layer="91"/>
<junction x="64.77" y="39.37"/>
<pinref part="U$3" gate="G$1" pin="VDD"/>
<wire x1="139.7" y1="65.024" x2="139.7" y2="39.37" width="0.1524" layer="91"/>
<wire x1="139.7" y1="39.37" x2="96.52" y2="39.37" width="0.1524" layer="91"/>
<junction x="96.52" y="39.37"/>
<pinref part="U$4" gate="G$1" pin="VIN"/>
<wire x1="182.88" y1="65.278" x2="182.88" y2="39.37" width="0.1524" layer="91"/>
<wire x1="182.88" y1="39.37" x2="139.7" y2="39.37" width="0.1524" layer="91"/>
<junction x="139.7" y="39.37"/>
<pinref part="U$5" gate="G$1" pin="VIN"/>
<wire x1="223.52" y1="66.04" x2="223.52" y2="39.37" width="0.1524" layer="91"/>
<wire x1="223.52" y1="39.37" x2="212.09" y2="39.37" width="0.1524" layer="91"/>
<junction x="182.88" y="39.37"/>
<pinref part="U$5" gate="G$1" pin="ADDR"/>
<wire x1="212.09" y1="39.37" x2="182.88" y2="39.37" width="0.1524" layer="91"/>
<wire x1="233.68" y1="96.52" x2="233.68" y2="101.6" width="0.1524" layer="91"/>
<wire x1="233.68" y1="101.6" x2="212.09" y2="101.6" width="0.1524" layer="91"/>
<wire x1="212.09" y1="101.6" x2="212.09" y2="39.37" width="0.1524" layer="91"/>
<junction x="212.09" y="39.37"/>
<label x="25.4" y="39.37" size="1.778" layer="95"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="9"/>
<pinref part="U$1" gate="G$1" pin="GND"/>
<wire x1="20.32" y1="36.83" x2="62.23" y2="36.83" width="0.1524" layer="91"/>
<wire x1="62.23" y1="36.83" x2="62.23" y2="62.992" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="GND"/>
<wire x1="101.6" y1="64.008" x2="101.6" y2="36.83" width="0.1524" layer="91"/>
<wire x1="101.6" y1="36.83" x2="62.23" y2="36.83" width="0.1524" layer="91"/>
<junction x="62.23" y="36.83"/>
<pinref part="U$3" gate="G$1" pin="GND"/>
<wire x1="142.24" y1="65.024" x2="142.24" y2="36.83" width="0.1524" layer="91"/>
<wire x1="142.24" y1="36.83" x2="101.6" y2="36.83" width="0.1524" layer="91"/>
<junction x="101.6" y="36.83"/>
<pinref part="U$4" gate="G$1" pin="GND"/>
<wire x1="187.96" y1="65.278" x2="187.96" y2="36.83" width="0.1524" layer="91"/>
<wire x1="187.96" y1="36.83" x2="142.24" y2="36.83" width="0.1524" layer="91"/>
<junction x="142.24" y="36.83"/>
<pinref part="U$5" gate="G$1" pin="GND"/>
<wire x1="228.6" y1="66.04" x2="228.6" y2="36.83" width="0.1524" layer="91"/>
<wire x1="228.6" y1="36.83" x2="187.96" y2="36.83" width="0.1524" layer="91"/>
<junction x="187.96" y="36.83"/>
<label x="25.4" y="36.83" size="1.778" layer="95"/>
</segment>
</net>
<net name="TX/TX1" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="8"/>
<pinref part="U$1" gate="G$1" pin="RX"/>
<wire x1="20.32" y1="34.29" x2="59.69" y2="34.29" width="0.1524" layer="91"/>
<wire x1="59.69" y1="34.29" x2="59.69" y2="62.992" width="0.1524" layer="91"/>
<label x="25.4" y="34.29" size="1.778" layer="95"/>
</segment>
</net>
<net name="RX/RX1" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="7"/>
<pinref part="U$1" gate="G$1" pin="TX"/>
<wire x1="20.32" y1="31.75" x2="57.15" y2="31.75" width="0.1524" layer="91"/>
<wire x1="57.15" y1="31.75" x2="57.15" y2="62.992" width="0.1524" layer="91"/>
<label x="25.4" y="31.75" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI_SCK" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="6"/>
<pinref part="U$2" gate="G$1" pin="CLK"/>
<wire x1="20.32" y1="29.21" x2="104.14" y2="29.21" width="0.1524" layer="91"/>
<wire x1="104.14" y1="29.21" x2="104.14" y2="64.008" width="0.1524" layer="91"/>
<label x="25.4" y="29.21" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI_MISO" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="5"/>
<pinref part="U$2" gate="G$1" pin="MISO"/>
<wire x1="20.32" y1="26.67" x2="106.68" y2="26.67" width="0.1524" layer="91"/>
<wire x1="106.68" y1="26.67" x2="106.68" y2="64.008" width="0.1524" layer="91"/>
<label x="25.4" y="26.67" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPI_MOSI" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="4"/>
<pinref part="U$2" gate="G$1" pin="MOSI"/>
<wire x1="20.32" y1="24.13" x2="109.22" y2="24.13" width="0.1524" layer="91"/>
<wire x1="109.22" y1="24.13" x2="109.22" y2="64.008" width="0.1524" layer="91"/>
<label x="25.4" y="24.13" size="1.778" layer="95"/>
</segment>
</net>
<net name="DIGITAL_PIN_5" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="3"/>
<pinref part="U$2" gate="G$1" pin="CS"/>
<wire x1="20.32" y1="21.59" x2="111.76" y2="21.59" width="0.1524" layer="91"/>
<wire x1="111.76" y1="21.59" x2="111.76" y2="64.008" width="0.1524" layer="91"/>
<label x="25.4" y="21.59" size="1.778" layer="95"/>
</segment>
</net>
<net name="I2C_SCL" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="2"/>
<pinref part="U$3" gate="G$1" pin="SCL"/>
<wire x1="20.32" y1="19.05" x2="144.78" y2="19.05" width="0.1524" layer="91"/>
<wire x1="144.78" y1="19.05" x2="144.78" y2="65.024" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="SCK"/>
<wire x1="190.5" y1="65.278" x2="190.5" y2="19.05" width="0.1524" layer="91"/>
<wire x1="190.5" y1="19.05" x2="144.78" y2="19.05" width="0.1524" layer="91"/>
<junction x="144.78" y="19.05"/>
<pinref part="U$5" gate="G$1" pin="SCL"/>
<wire x1="233.68" y1="66.04" x2="233.68" y2="19.05" width="0.1524" layer="91"/>
<wire x1="233.68" y1="19.05" x2="190.5" y2="19.05" width="0.1524" layer="91"/>
<junction x="190.5" y="19.05"/>
<label x="25.4" y="19.05" size="1.778" layer="95"/>
</segment>
</net>
<net name="I2C_SDA" class="0">
<segment>
<pinref part="MCU_PINS" gate="A" pin="1"/>
<pinref part="U$3" gate="G$1" pin="SDA"/>
<wire x1="20.32" y1="16.51" x2="147.32" y2="16.51" width="0.1524" layer="91"/>
<wire x1="147.32" y1="16.51" x2="147.32" y2="65.024" width="0.1524" layer="91"/>
<pinref part="U$4" gate="G$1" pin="MOSI"/>
<wire x1="195.58" y1="65.278" x2="195.58" y2="16.51" width="0.1524" layer="91"/>
<wire x1="195.58" y1="16.51" x2="147.32" y2="16.51" width="0.1524" layer="91"/>
<junction x="147.32" y="16.51"/>
<pinref part="U$5" gate="G$1" pin="SDA"/>
<wire x1="231.14" y1="66.04" x2="231.14" y2="16.51" width="0.1524" layer="91"/>
<wire x1="231.14" y1="16.51" x2="195.58" y2="16.51" width="0.1524" layer="91"/>
<junction x="195.58" y="16.51"/>
<label x="25.4" y="16.51" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="6.3" minversion="6.2.2" severity="warning">
Since Version 6.2.2 text objects can contain more than one line,
which will not be processed correctly with this version.
</note>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
