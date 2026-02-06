#!/bin/bash

echo "--------------------------------------------------------"
echo "       ROS 2 TELEOP ROBOT - USB PERSISTENCE SETUP       "
echo "--------------------------------------------------------"
echo "This script will help you assign a permanent name to your LiDAR."
echo "No matter which port you plug it into, it will be: /dev/lidar"
echo ""

# 1. Detect LiDAR
echo "🔌 Step 1: Unplug the LiDAR from the USB port."
read -p "Press Enter when it is UNPLUGGED..."

echo "Scanning current devices..."
ls /dev/ttyUSB* > /tmp/usb_before 2>/dev/null

echo ""
echo "🔌 Step 2: PLUG IN the LiDAR now."
read -p "Press Enter after you have plugged it in..."

ls /dev/ttyUSB* > /tmp/usb_after 2>/dev/null
NEW_DEV=$(comm -13 /tmp/usb_before /tmp/usb_after)

if [ -z "$NEW_DEV" ]; then
    echo "❌ Error: No new device detected!"
    echo "Did you plug it in? Is the cable good? Try running the script again."
    exit 1
fi

echo "✅ Detected new device: $NEW_DEV"

# 2. Get Attributes
echo "🔍 Analyzing device..."
# Get ID_VENDOR and ID_PRODUCT_ID
ID_VENDOR=$(udevadm info --name=$NEW_DEV --attribute-walk | grep "ATTRS{idVendor}" | head -n 1 | cut -d '"' -f 2)
ID_PRODUCT=$(udevadm info --name=$NEW_DEV --attribute-walk | grep "ATTRS{idProduct}" | head -n 1 | cut -d '"' -f 2)

if [ -z "$ID_VENDOR" ] || [ -z "$ID_PRODUCT" ]; then
    echo "❌ Error: Could not read Vendor/Product ID."
    exit 1
fi

echo "   Vendor ID:  $ID_VENDOR"
echo "   Product ID: $ID_PRODUCT"

# 3. Write Rule
RULE_FILE="/etc/udev/rules.d/99-rover-lidar.rules"
echo ""
echo "📝 Creating Rule File: $RULE_FILE"

# Create the rule line
RULE_LINE="KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"$ID_VENDOR\", ATTRS{idProduct}==\"$ID_PRODUCT\", SYMLINK+=\"lidar\", MODE=\"0777\""

echo "   Writing: $RULE_LINE"
echo "$RULE_LINE" | sudo tee $RULE_FILE > /dev/null

# 4. Reload
echo "🔄 Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "--------------------------------------------------------"
echo "✅ SUCCESS! Your LiDAR is now: /dev/lidar"
echo "--------------------------------------------------------"
echo "ℹ️  Update your launch file to use '/dev/lidar' instead of '/dev/ttyUSB0'."
