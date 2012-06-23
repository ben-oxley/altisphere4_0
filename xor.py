print "Please enter your string to XOR:"
crc = 0
data = raw_input()
for i in range(0,len(data)):
	crc ^= ord(data[i])
print "The HEX value for the checksum is:"
print hex(crc)
print "Overall, the sentence will be:"
print "$"+data+"*"+hex(crc).lstrip("0x")
