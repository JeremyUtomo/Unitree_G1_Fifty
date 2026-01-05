# SSH Key Setup for Integrated Controller

The integrated controller requires passwordless SSH access to the robot to run the auto-centering script remotely.

## Robot Details
- **Host**: unitree@192.168.123.164
- **Password**: 123
- **Working Directory**: /home/unitree/Unitree_G1_Fifty

## Setup SSH Keys (One-Time Setup)

### 1. Generate SSH Key Pair (if you don't have one)

On your laptop:
```bash
ssh-keygen -t ed25519 -C "unitree_g1_controller"
```

When prompted:
- **File location**: Press Enter (use default: `~/.ssh/id_ed25519`)
- **Passphrase**: Press Enter twice (no passphrase for automated scripts)

### 2. Copy Public Key to Robot

Use `ssh-copy-id` to automatically install your public key:

```bash
ssh-copy-id unitree@192.168.123.164
```

When prompted:
- **Password**: `123`

This will copy your public key to the robot's `~/.ssh/authorized_keys` file.

### 3. Test SSH Connection

Verify passwordless SSH works:

```bash
ssh unitree@192.168.123.164 'echo "SSH key working!"'
```

You should see `SSH key working!` without being prompted for a password.

### 4. Test Remote Script Execution

Test running a command on the robot:

```bash
ssh unitree@192.168.123.164 'cd /home/unitree/Unitree_G1_Fifty && pwd'
```

Expected output: `/home/unitree/Unitree_G1_Fifty`

## Troubleshooting

### "Permission denied (publickey,password)"

If SSH still asks for password:

1. **Check SSH key permissions on laptop:**
   ```bash
   chmod 700 ~/.ssh
   chmod 600 ~/.ssh/id_ed25519
   chmod 644 ~/.ssh/id_ed25519.pub
   ```

2. **Check authorized_keys on robot:**
   ```bash
   ssh unitree@192.168.123.164
   # Enter password: 123
   chmod 700 ~/.ssh
   chmod 600 ~/.ssh/authorized_keys
   exit
   ```

3. **Verify key is installed:**
   ```bash
   ssh unitree@192.168.123.164 'cat ~/.ssh/authorized_keys'
   ```
   
   You should see your public key (starts with `ssh-ed25519 ...`)

### "Could not resolve hostname"

Check network connectivity:
```bash
ping 192.168.123.164
```

If ping fails, verify:
- Robot is powered on
- Network interface is connected (e.g., `enp49s0`)
- IP configuration is correct

### SSH Key Already Exists

If you already have SSH keys and want to use them:

```bash
# Just copy your existing public key
ssh-copy-id -i ~/.ssh/id_rsa.pub unitree@192.168.123.164
```

Or manually append to authorized_keys:
```bash
cat ~/.ssh/id_rsa.pub | ssh unitree@192.168.123.164 'mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys'
```

## Alternative: Manual Key Installation

If `ssh-copy-id` doesn't work:

1. **Get your public key:**
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```

2. **Copy the output** (entire line starting with `ssh-ed25519`)

3. **SSH to robot and add key:**
   ```bash
   ssh unitree@192.168.123.164
   # Password: 123
   
   mkdir -p ~/.ssh
   chmod 700 ~/.ssh
   echo "PASTE_YOUR_PUBLIC_KEY_HERE" >> ~/.ssh/authorized_keys
   chmod 600 ~/.ssh/authorized_keys
   exit
   ```

4. **Test connection:**
   ```bash
   ssh unitree@192.168.123.164 'echo "Success!"'
   ```

## Security Note

⚠️ **Important**: The SSH key generated has no passphrase for automated script execution. This is required for the integrated controller to work without manual intervention.

If you need better security:
- Use SSH agent with `ssh-add` to cache passphrase
- Restrict key usage with `command=` in `authorized_keys`
- Use dedicated SSH key only for this robot

## Verification Checklist

Before running `integrated_controller.py`, verify:

- [ ] SSH key is set up (no password prompt)
- [ ] Can SSH to robot: `ssh unitree@192.168.123.164`
- [ ] Remote script path exists: `/home/unitree/Unitree_G1_Fifty/src/center_bottle/auto_center_bottle.py`
- [ ] YOLO model is available on robot: `/home/unitree/Unitree_G1_Fifty/src/center_bottle/yolov8n.pt`
- [ ] ROS2 environment is sourced (for `/bottle_alignment_status` topic)

## Next Steps

Once SSH keys are set up, you can run the integrated controller:

```bash
cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
source setup_slam.sh
python3 src/integrated_controller.py enp49s0
```
