#ifndef MCUXMEMPROXY_H
#define MCUXMEMPROXY_H

#include "rammemoryproxy.h"
#include "avrcore.h"
#include "e_mcu.h"
#include "mcuport.h"
#include <QDebug>

class McuXmemProxy : public RamMemoryProxy
{
public:
    McuXmemProxy(AvrCore* avrCore, eMcu* mcu, uint8_t* ram, uint32_t ramSize, uint8_t* xmcra = nullptr, uint8_t* xmcrb = nullptr)
        : m_avrCore(avrCore)
        , m_mcu(mcu)
        , m_ram(ram)
        , m_ramSize(ramSize)
        , m_xmcra(xmcra)
        , m_xmcrb(xmcrb)
        , m_sre(false)
        , m_srw0(0)
        , m_srw1(0)
    {}

    uint8_t read(uint16_t address) override
    {
        if (address < 0x2200) {
            if (m_ram && address < m_ramSize) {
                return m_ram[address];
            }
            return 0xFF;
        }

        if (isXmemEnabled()) {
            uint8_t data = 0xFF;
            triggerBusCycle(address, data, false);
            return data;
        }

        return 0xFF;
    }

    void write(uint16_t address, uint8_t value) override
    {
        // 1. Intercept XMCRA (0x74) and XMCRB (0x75)
        if (address == 0x74 || address == 0x0074) {
            qDebug() << "[XMEM-PROXY] XMCRA configured with:" << Qt::hex << value;
            updateXmemRegisters(0x74, value);
            if (m_ram && address < m_ramSize) m_ram[address] = value;
            return;
        } 
        if (address == 0x75 || address == 0x0075) {
            updateXmemRegisters(0x75, value);
            if (m_ram && address < m_ramSize) m_ram[address] = value;
            return;
        }

        // 2. Internal SRAM & Registers (< 0x2200)
        if (address < 0x2200) {
            if (m_ram && address < m_ramSize) {
                m_ram[address] = value;
            }
            return;
        }

        // 3. External Memory Space (> 0x21FF)
        if (isXmemEnabled()) {
            qDebug() << "[XMEM-PROXY] External XMEM Write at address:" << Qt::hex << address << "Data:" << value;
            uint8_t valToWrite = value;
            triggerBusCycle(address, valToWrite, true);
            m_externalRamSimulation[address & 0xFFFF] = value;
        }
    }

    void reset() override {
        m_sre = false;
        m_srw0 = 0;
        m_srw1 = 0;
        m_externalRamSimulation.clear();
    }

private:
    bool isXmemEnabled() const {
        if (m_xmcra) {
            return (*m_xmcra & 0x80) != 0; // SRE bit
        }
        return m_sre;
    }

    void updateXmemRegisters(uint16_t regAddr, uint8_t val) {
        if (regAddr == 0x74) {
            m_sre = (val & 0x80) != 0;
            // SRW00/SRW01 are bits 3:2, SRW10/SRW11 are bits 1:0 in XMCRA for lower/upper sectors
            m_srw0 = (val >> 2) & 0x03;
            m_srw1 = val & 0x03;
            qDebug() << "[XMEM-REG] XMCRA: SRE=" << m_sre << "SRW0=" << m_srw0 << "SRW1=" << m_srw1;
        }
    }


    void triggerBusCycle(uint16_t address, uint8_t& data, bool isWrite) {
        uint8_t lowAddr = static_cast<uint8_t>(address & 0xFF);
        uint8_t highAddr = static_cast<uint8_t>((address >> 8) & 0xFF);

        if (!m_mcu || !m_avrCore) return;

	// --- DEBUG: Inspect Port G and its Pins before starting cycle ---
        McuPort* portG = m_mcu->getMcuPort("PORTG");
        if (portG) {
            McuPin* p0 = portG->getPinN(0); // /RD
            McuPin* p1 = portG->getPinN(1); // /WR
            McuPin* p2 = portG->getPinN(2); // ALE
            
            qDebug() << "[XMEM-DEBUG] Entering Bus Cycle for Addr:" << Qt::hex << address
                     << "PORTG reg val:" << m_ram[0x34]
                     << "PG0 connected:" << (p0 ? p0->isConnected() : false)
                     << "PG1 connected:" << (p1 ? p1->isConnected() : false)
                     << "PG2 connected:" << (p2 ? p2->isConnected() : false);
        }

        uint8_t waitStates = (address >= 0x8000) ? m_srw1 : m_srw0;

        // 1. Configure Port Directions (DDR)
        m_avrCore->writePortReg(0x27, 0xFF); // DDRC out
        m_avrCore->writePortReg(0x21, isWrite ? 0xFF : 0x00); // DDRA out/in

        uint8_t ddrg = m_ram[0x33];
        ddrg |= (1 << 0) | (1 << 1) | (1 << 2); // PG0(/RD), PG1(/WR), PG2(ALE)
        m_avrCore->writePortReg(0x33, ddrg);

        // 2. Drive Address Bus (High byte on PORTC, Low byte on PORTA)
        m_avrCore->writePortReg(0x28, highAddr);
        m_avrCore->writePortReg(0x22, 0); // Low byte address initially 0 for ALE

        uint8_t portgVal = m_ram[0x34];
        portgVal |= (1 << 0) | (1 << 1);
        portgVal &= ~(1 << 2);

	auto setPortG = [&](uint8_t val, int simulatedCycles ) {
            m_avrCore->writePortReg(0x34, val);
	    m_mcu->cyclesDone += simulatedCycles;
            qDebug() << "[XMEM-DEBUG] PORTG set to:" << Qt::hex << val 
                     << "Logic states -> RD:" << ((val & 1) ? 1 : 0) 
                     << "WR:" << ((val & 2) ? 1 : 0) 
                     << "ALE:" << ((val & 4) ? 1 : 0);
        };

        // --- Phase 1: ALE High & Low ---
        portgVal |= (1 << 2); // ALE High
        setPortG(portgVal,1);
	McuPin* alePin = m_mcu->getMcuPin("PG2"); // or verify exact pin ID
        if (alePin) {
	    alePin->setOutState(true);  // Drive high/low directly
	}

        portgVal &= ~(1 << 2); // ALE Low (Address Latched)
        setPortG(portgVal,1);

        // --- Phase 2: Strobe Activation ---
        if (isWrite) {
            m_avrCore->writePortReg(0x22, data); // Put data on PORTA
            portgVal &= ~(1 << 1); // /WR Low
            setPortG(portgVal,1);
        } else {
            m_avrCore->writePortReg(0x21, 0x00); // Input mode for read on DDRA
            portgVal &= ~(1 << 0); // /RD Low
            setPortG(portgVal,1);
        }

        m_mcu->cyclesDone += waitStates;

        // --- Phase 3: Strobe De-assertion ---
        portgVal |= (1 << 0) | (1 << 1); // Return /RD and /WR High
        setPortG(portgVal,1);

        if (!isWrite) {
            data = m_ram[0x20]; // Read from PORTA input pins register
            m_avrCore->writePortReg(0x21, 0xFF); // Restore DDRA
        }

        m_mcu->cyclesDone += 2;
    }

    AvrCore* m_avrCore;
    eMcu* m_mcu;
    uint8_t* m_ram;
    uint32_t m_ramSize;
    uint8_t* m_xmcra;
    uint8_t* m_xmcrb;
    bool m_sre;
    uint8_t m_srw0;
    uint8_t m_srw1;
    QHash<uint16_t, uint8_t> m_externalRamSimulation;
};

#endif // MCUXMEMPROXY_H
       //
