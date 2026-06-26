-- Direct Drive Lua Script s komunikací a uzavřenou smyčkou pro RPM
-- Rozhraní: 
-- ping -> pong\n ACK\n
-- current <mA> -> ACK\n
-- rpm <levy> <pravy> -> ACK\n
-- speed <levy_ms> <pravy_ms> -> ACK\n
-- stop -> ACK\n

local target_rpm_L = 0
local target_rpm_R = 0
local max_current_mA = 5000

local kp = 0.01
local last_serial_msg_time = millis()

local KOLO_POLOMER = 0.22
local PREVOD = 25.0       
local OBVOD_KOLA = 2.0 * math.pi * KOLO_POLOMER
local SPEED_TO_RPM = (60 * PREVOD) / OBVOD_KOLA -- Přepočet rychlosti na otáčky motoru (cca 1085.15)

local usb_port = serial:find_serial(2)
local buffer = ""

if not usb_port then
    gcs:send_text(4, "DIR-DRIVE: UART 2 (Scripting) nenalezen!")
else
    usb_port:begin(115200)
    gcs:send_text(6, "DIR-DRIVE: Spusteno na UART 2")
end

local function constrain(val, min_val, max_val)
    if val < min_val then return min_val end
    if val > max_val then return max_val end
    return val
end

local function get_num(val)
    if not val then return 0.0 end
    if type(val) == "userdata" and val.tofloat then 
        return val:tofloat() 
    end
    return tonumber(val) or 0.0
end

local function parse_serial()
    if not usb_port then return end
    local n_bytes = usb_port:available():toint()
    if n_bytes > 0 then
        local data = usb_port:readstring(n_bytes)
        if data then
            buffer = buffer .. data
            
            while true do
                local newline_idx = string.find(buffer, "\n")
                if not newline_idx then break end
                
                local line = string.sub(buffer, 1, newline_idx - 1)
                buffer = string.sub(buffer, newline_idx + 1)
                line = string.gsub(line, "\r", "")
                
                local ack = false
                local handled = false
                local custom_reply = nil

                if line == "ping" then
                    usb_port:writestring("PONG DIRECT\n")
                    ack = true
                    handled = true
                elseif string.sub(line, 1, 4) == "stop" then
                    target_rpm_L = 0
                    target_rpm_R = 0
                    gcs:send_text(6, "DIR-DRIVE: STOP")
                    ack = true
                    handled = true
                elseif string.sub(line, 1, 7) == "current" then
                    local _, _, c = string.find(line, "current%s+(%S+)")
                    if c then
                        max_current_mA = tonumber(c) or max_current_mA
                        gcs:send_text(6, string.format("DIR-DRIVE: Current limit %dmA", max_current_mA))
                        ack = true
                        handled = true
                    end
                elseif string.sub(line, 1, 3) == "rpm" then
                    local _, _, r_l, r_r = string.find(line, "rpm%s+(%S+)%s+(%S+)")
                    if r_l and r_r then
                        if not arming:is_armed() then
                            custom_reply = "NACK: Not armed\n"
                            handled = true
                        else
                            target_rpm_L = constrain(tonumber(r_l) or 0, -3000, 3000)
                            target_rpm_R = constrain(tonumber(r_r) or 0, -3000, 3000)
                            last_serial_msg_time = millis()
                            gcs:send_text(6, string.format("DIR-DRIVE: RPM L:%d R:%d", target_rpm_L, target_rpm_R))
                            ack = true
                            handled = true
                        end
                    end
                elseif string.sub(line, 1, 5) == "speed" then
                    local _, _, s_l, s_r = string.find(line, "speed%s+(%S+)%s+(%S+)")
                    if s_l and s_r then
                        if not arming:is_armed() then
                            custom_reply = "NACK: Not armed\n"
                            handled = true
                        else
                            local speed_l = tonumber(s_l) or 0
                            local speed_r = tonumber(s_r) or 0
                            target_rpm_L = constrain(speed_l * SPEED_TO_RPM, -3000, 3000)
                            target_rpm_R = constrain(speed_r * SPEED_TO_RPM, -3000, 3000)
                            last_serial_msg_time = millis()
                            gcs:send_text(6, string.format("DIR-DRIVE: SPEED L:%.2f R:%.2f", speed_l, speed_r))
                            ack = true
                            handled = true
                        end
                    end
                end

                if handled then
                    if custom_reply then
                        usb_port:writestring(custom_reply)
                    elseif ack then
                        usb_port:writestring("ACK\n")
                    else
                        usb_port:writestring("NACK\n")
                    end
                elseif string.len(line) > 0 then
                    usb_port:writestring("NACK\n")
                end
            end
            
            if string.len(buffer) > 200 then
                buffer = ""
            end
        end
    end
end

local function calc_pwm(target_rpm, current_rpm, current_mA)
    if math.abs(target_rpm) < 10 then
        return 1500
    end

    -- Otáčky z RPM senzoru jsou obvykle absolutní hodnota, dáme jim znaménko podle targetu
    local dir = (target_rpm > 0) and 1 or -1
    local signed_current_rpm = current_rpm * dir

    local err = target_rpm - signed_current_rpm
    
    -- Omezení podle max_current
    if current_mA > max_current_mA then
        local over_current = current_mA - max_current_mA
        -- Snižujeme tah úměrně k překročenému proudu
        if target_rpm > 0 then
            err = err - (over_current * 0.5)
        else
            err = err + (over_current * 0.5)
        end
    end

    -- Přesný Feedforward vypočítaný z trace.log! 
    -- Z toho plyne: Mrtvá zóna je 47 PWM, a nárůst je 0.061 PWM na 1 RPM.
    local ff_pwm = (dir * 47) + (target_rpm * 0.061)
    
    -- Místo integrátoru (který způsoboval windup a couvání) použijeme pouze Proporcionální složku (jako v drive.c)
    local p_pwm = kp * err
    
    -- Omezení maximální opravy od PI regulátoru pro bezpečnost
    p_pwm = constrain(p_pwm, -100, 100)

    local total_pwm = math.floor(1500 + ff_pwm + p_pwm)
    
    -- ZABRÁNĚNÍ NECHTĚNÉHO COUVÁNÍ: 
    -- Regulátor by mohl při prudkém zpomalení vygenerovat PWM pro couvání (<1500 u dopředné jízdy).
    -- Proto PWM ořízneme tak, aby nikdy nešlo "proti" požadovanému směru jízdy.
    if dir > 0 then
        total_pwm = constrain(total_pwm, 1500, 2000)
    else
        total_pwm = constrain(total_pwm, 1000, 1500)
    end
    
    -- Ochrana mrtvé zóny (okolo středu 1500 nedáváme signál, aby motory nepískaly)
    if total_pwm > 1485 and total_pwm < 1515 then
        total_pwm = 1500
    end

    return total_pwm
end

function update()
    parse_serial()

    local is_armed = arming:is_armed()
    local rc_ok = rc:has_valid_input()

    -- 1. Detekce krizových událostí a VYNUCENÍ DISARMU
    if is_armed then
        if not rc_ok then
            arming:disarm()
            gcs:send_text(0, "DIR-DRIVE: Ztrata RC signalu -> DISARM!")
            is_armed = false
        end
    end

    -- 2. Zastavení motorů (pokud nejsme Armed)
    if not is_armed then
        -- Vynulujeme cílové rychlosti
        target_rpm_L = 0
        target_rpm_R = 0
        
        -- Zastavíme serva (neutrál 1500)
        SRV_Channels:set_output_pwm_chan_timeout(0, 1500, 100)
        SRV_Channels:set_output_pwm_chan_timeout(1, 1500, 100)
        
        -- Vyčistíme případná stará data z bufferu od Pythonu, aby se nerozjel hned po armnutí
        if usb_port and usb_port:available():toint() > 0 then
            usb_port:readstring(usb_port:available():toint())
        end
        buffer = "" -- DŮLEŽITÉ: musíme smazat i Lua buffer, aby nezůstala načtená půlka zprávy
        
        return update, 20
    end

    -- RPM čteme křížem dle posledního zadání (L=0, R=1)
    local current_rpm_L = get_num(RPM:get_rpm(0))
    local current_rpm_R = get_num(RPM:get_rpm(1))

    -- MAVSense Telemetry: Levý=0, Pravý=1
    local curr_mA_L = get_num(esc_telem:get_current(0)) * 1000
    local curr_mA_R = get_num(esc_telem:get_current(1)) * 1000

    local t_rpm_L = target_rpm_L
    local t_rpm_R = target_rpm_R

    -- RC OVERRIDE LOGIKA
    -- Ch 1 = RC_IN_0 (Zatáčení), Ch 2 = RC_IN_1 (Plyn), Ch 3 = RC_IN_2 (Přepínač)
    local rc_switch = rc:get_pwm(3) or 1000
    if rc_switch > 1500 then
        -- Pokud řídíme přes RC, mažeme z paměti staré USB příkazy.
        -- Zabrání to tomu, aby se auto po přepnutí zpět na USB rozjelo původní rychlostí.
        target_rpm_L = 0
        target_rpm_R = 0
        
        local steer_pwm = rc:get_pwm(1) or 1500
        local throt_pwm = rc:get_pwm(2) or 1500

        -- Normalizace <-1.0, 1.0>
        local steer = (steer_pwm - 1500) / 500
        local throt = (throt_pwm - 1500) / 500

        -- Deadzone pro páčky vysílačky (aby v neutrálu nehrčely)
        if math.abs(steer) < 0.05 then steer = 0 end
        if math.abs(throt) < 0.05 then throt = 0 end

        -- Max RPM dosažitelné z vysílače (přibližně 1 m/s)
        local MAX_RC_RPM = 1000 

        -- Skid steering mix
        -- Páka doprava (steer > 0) zrychlí levé kolo a zpomalí pravé
        local raw_L = throt + steer
        local raw_R = throt - steer

        -- Zachování správného poměru zatáčení při plném plynu (zabráníme ořezu asymetricky)
        local max_raw = math.max(1.0, math.abs(raw_L), math.abs(raw_R))
        t_rpm_L = (raw_L / max_raw) * MAX_RC_RPM
        t_rpm_R = (raw_R / max_raw) * MAX_RC_RPM
    end

    local pwm_L = calc_pwm(t_rpm_L, current_rpm_L, curr_mA_L)
    local pwm_R = calc_pwm(t_rpm_R, current_rpm_R, curr_mA_R)

    SRV_Channels:set_output_pwm_chan_timeout(0, pwm_L, 100)
    SRV_Channels:set_output_pwm_chan_timeout(1, pwm_R, 100)

    return update, 20
end

return update, 1000