#ifndef MCUXMEMPROXY_H
#define MCUXMEMPROXY_H

#include "rammemoryproxy.h"
#include "avrcore.h"
#include "e_mcu.h"
#include "mcuport.h"
#include "simulator.h"
#include "e-element.h"
#include <QDebug>

class McuXmemProxy : public RamMemoryProxy, public eElement
{
public:
    McuXmemProxy(AvrCore* avrCore, eMcu* mcu, uint8_t* ram, uint32_t ramSize, uint8_t* xmcra = nullptr, uint8_t* xmcrb = nullptr)
        : eElement(mcu ? mcu->getId() + "-XmemProxy" : "XmemProxy")
        , m_avrCore(avrCore)
        , m_mcu(mcu)
        , m_ram(ram)
        , m_ramSize(ramSize)
        , m_xmcra(xmcra)
        , m_xmcrb(xmcrb)
        , m_sre(false)
        , m_srw0(0)
        , m_srw1(0)
        , m_rdPin(nullptr)
        , m_wrPin(nullptr)
        , m_alePin(nullptr)
        , m_dataPort(nullptr)
        , m_psStep(0)
        , m_addrSetTime(0)
        , m_laEnEndTime(0)
        , m_readSetTime(0)
        , m_writeSetTime(0)
        , m_readBusTime(0)
        , m_dataTime(0)
        , m_xmemState(xmem_IDLE)
        , m_targetAddress(0)
        , m_targetData(nullptr)
        , m_isWrite(false)
        , m_portgVal(0)
    {
        if (m_mcu) {
            m_rdPin    = m_mcu->getMcuPin("PG0"); // /RD Pin
            m_wrPin    = m_mcu->getMcuPin("PG1"); // /WR Pin
            m_alePin   = m_mcu->getMcuPin("PG2"); // ALE Pin
            m_dataPort = m_mcu->getMcuPort("PORTA"); // PORTA como barramento de dados/endereço baixo

            uint64_t basePsInst = 62500; // Fallback para 16 MHz

            uint64_t cycleTime = basePsInst * 4; 
            m_psStep       = cycleTime / 12;
            m_addrSetTime  = 3 * m_psStep;
            m_laEnEndTime  = 4 * m_psStep;
            m_readSetTime  = 6 * m_psStep;
            m_writeSetTime = 6 * m_psStep;
            m_readBusTime  = (cycleTime > 10) ? (cycleTime - 10) : 0;

            qDebug() << "[XMEM-TIMING-INIT] AVR Core (4 clocks/inst) | BasePsInst:" << basePsInst
                     << "CycleTime:" << cycleTime
                     << "psStep:" << m_psStep
                     << "addrSetTime:" << m_addrSetTime
                     << "laEnEndTime:" << m_laEnEndTime
                     << "readSetTime:" << m_readSetTime
                     << "writeSetTime:" << m_writeSetTime
                     << "readBusTime:" << m_readBusTime;
        }
    }
    
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

        if (address < 0x2200) {
            if (m_ram && address < m_ramSize) {
                m_ram[address] = value;
            }
            return;
        }

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
        m_xmemState = xmem_IDLE;
        m_externalRamSimulation.clear();
        Simulator::self()->cancelEvents(this);

        if (m_mcu) {
            uint64_t cycleTime = m_mcu->psInst(); 
            m_psStep       = cycleTime / 12;
            m_addrSetTime  = 3 * m_psStep;
            m_laEnEndTime  = 4 * m_psStep;
            m_readSetTime  = 6 * m_psStep;
            m_writeSetTime = 6 * m_psStep;
            m_readBusTime  = cycleTime - 10;

            qDebug() << "[XMEM-TIMING-RESET] CycleTime(psInst):" << cycleTime
                     << "addrSetTime:" << m_addrSetTime
                     << "readSetTime:" << m_readSetTime
                     << "writeSetTime:" << m_writeSetTime;
        }
    }

    void runEvent() 
    {
        uint64_t cycleTime = (m_mcu && m_mcu->psInst() > 0) ? (m_mcu->psInst() * 4) : 250000;
        uint64_t psStep = cycleTime / 12;

        switch (m_xmemState)
        {
            case xmem_IDLE:
                qDebug() << "[XMEM] State: IDLE - Aguardando novo ciclo.";
                break;

            case xmem_ALE_HIGH:
            {
                m_portgVal |= (1 << 2); 
                if (m_alePin) { 
                    m_alePin->setOutState(true); 
                    m_alePin->updateStep(); 
                }

                uint8_t lowAddr = m_targetAddress & 0xFF;
                if (m_dataPort) {
                    m_dataPort->outChanged(lowAddr);
                }

                qDebug() << "[XMEM] State: ALE_HIGH | Endereço Alvo:" << Qt::hex << m_targetAddress 
                         << "| LowAddr no m_dataPort (PORTA):" << Qt::hex << lowAddr << "| ALE pin set to HIGH";

                Simulator::self()->addEvent(4 * psStep, this);
                m_xmemState = xmem_ALE_LOW;
            }
            break;

            case xmem_ALE_LOW:
            {
                m_portgVal &= ~(1 << 2);
                if (m_alePin) { 
                    m_alePin->setOutState(false); 
                    m_alePin->updateStep(); 
                }

                if (m_isWrite) {
                    if (m_dataPort && m_targetData) {
                        m_dataPort->outChanged(*m_targetData);
                    }
                    qDebug() << "[XMEM] State: ALE_LOW (WRITE) | Dado a escrever no m_dataPort:" << Qt::hex << *m_targetData;
                } else {
                    if (m_dataPort) {
                        m_dataPort->outChanged(0xFF);
                    }
                    qDebug() << "[XMEM] State: ALE_LOW (READ) | m_dataPort configurado para alta impedância / pull-ups.";
                }

                Simulator::self()->addEvent(2 * psStep, this);
                m_xmemState = xmem_STROBE_ACTIVE;
            }
            break;

            case xmem_STROBE_ACTIVE:
            {
                if (m_isWrite) {
                    if (m_wrPin) { 
                        m_wrPin->setOutState(false); 
                        m_wrPin->updateStep(); 
                    }
                    qDebug() << "[XMEM] State: STROBE_ACTIVE (WRITE) | /WR pin set to LOW";
                } else {
                    if (m_rdPin) { 
                        m_rdPin->setOutState(false); 
                        m_rdPin->updateStep(); 
                    }
                    qDebug() << "[XMEM] State: STROBE_ACTIVE (READ) | /RD pin set to LOW";
                }

                Simulator::self()->addEvent(4 * psStep, this);
                m_xmemState = xmem_STROBE_HOLD;
            }
            break;

            case xmem_STROBE_HOLD:
            {
                if (!m_isWrite) {
                    if (m_targetData && m_dataPort) {
                        *m_targetData = m_dataPort->getInpState();
                    }
                    qDebug() << "[XMEM] State: STROBE_HOLD (READ) | Dado lido do m_dataPort:" << Qt::hex << (m_targetData ? *m_targetData : 0);
                } else {
                    qDebug() << "[XMEM] State: STROBE_HOLD (WRITE) | Retenção de escrita concluída.";
                }

                Simulator::self()->addEvent(2 * psStep, this);
                m_xmemState = xmem_STROBE_END;
            }
            break;

            case xmem_STROBE_END:
            {
                if (m_isWrite) {
                    if (m_wrPin) { 
                        m_wrPin->setOutState(true); 
                        m_wrPin->updateStep(); 
                    }
                    qDebug() << "[XMEM] /WR pin set to HIGH";
                } else {
                    if (m_rdPin) { 
                        m_rdPin->setOutState(true); 
                        m_rdPin->updateStep(); 
                    }
                    qDebug() << "[XMEM] /RD pin set to HIGH";
                }

                qDebug() << "[XMEM] Ciclo XMEM concluído com sucesso.";
                m_xmemState = xmem_IDLE;
            }
            break;

            default:
                qDebug() << "[XMEM] ERRO: Estado desconhecido na máquina de estados. A redefinir para IDLE.";
                m_xmemState = xmem_IDLE;
                break;
        }
    }

private:
    bool isXmemEnabled() const {
        if (m_xmcra) {
            return (*m_xmcra & 0x80) != 0;
        }
        return m_sre;
    }

    void updateXmemRegisters(uint16_t regAddr, uint8_t val) {
        if (regAddr == 0x74) {
            m_sre = (val & 0x80) != 0;
            m_srw0 = (val >> 2) & 0x03;
            m_srw1 = val & 0x03;
            qDebug() << "[XMEM-REG] XMCRA: SRE=" << m_sre << "SRW0=" << m_srw0 << "SRW1=" << m_srw1;
        }
    }

    void triggerBusCycle(uint16_t address, uint8_t& data, bool isWrite) {
        uint8_t lowAddr = static_cast<uint8_t>(address & 0xFF);
        uint8_t highAddr = static_cast<uint8_t>((address >> 8) & 0xFF);

        if (!m_mcu || !m_avrCore) return;

        qDebug() << "[XMEM-DEBUG] Entering Bus Cycle for Addr:" << Qt::hex << address
                 << "PORTG reg val:" << (m_ram ? m_ram[0x34] : 0)
                 << "PG0(/RD) connected:" << (m_rdPin ? m_rdPin->isConnected() : false)
                 << "PG1(/WR) connected:" << (m_wrPin ? m_wrPin->isConnected() : false)
                 << "PG2(ALE) connected:" << (m_alePin ? m_alePin->isConnected() : false);

        m_targetAddress = address;
        m_targetData = &data;
        m_isWrite = isWrite;

        m_avrCore->writePortReg(0x27, 0xFF); // DDRC out
        m_avrCore->writePortReg(0x21, isWrite ? 0xFF : 0x00); // DDRA out/in

        uint8_t ddrg = m_ram ? m_ram[0x33] : 0;
        ddrg |= (1 << 0) | (1 << 1) | (1 << 2);
        m_avrCore->writePortReg(0x33, ddrg);

        m_avrCore->writePortReg(0x28, highAddr);
        m_avrCore->writePortReg(0x22, lowAddr); 

        m_portgVal = m_ram ? m_ram[0x34] : 0;
        m_portgVal |= (1 << 0) | (1 << 1);
        m_portgVal &= ~(1 << 2);

        m_xmemState = xmem_ALE_HIGH;
        Simulator::self()->addEvent(1, this);
    }

    enum xmemState_t {
        xmem_IDLE = 0,
        xmem_ALE_HIGH,
        xmem_ALE_LOW,
        xmem_STROBE_ACTIVE,
        xmem_STROBE_HOLD,
        xmem_STROBE_END
    };

    AvrCore* m_avrCore;
    eMcu* m_mcu;
    uint8_t* m_ram;
    uint32_t m_ramSize;
    uint8_t* m_xmcra;
    uint8_t* m_xmcrb;
    bool m_sre;
    uint8_t m_srw0;
    uint8_t m_srw1;

    McuPin*  m_rdPin;
    McuPin*  m_wrPin;
    McuPin*  m_alePin;
    McuPort* m_dataPort; // Adicionado para controlo direto do porto de dados

    uint64_t m_psStep;
    uint64_t m_addrSetTime;
    uint64_t m_laEnEndTime;
    uint64_t m_readSetTime;
    uint64_t m_writeSetTime;
    uint64_t m_readBusTime;
    uint64_t m_dataTime;

    xmemState_t m_xmemState;
    uint16_t m_targetAddress;
    uint8_t* m_targetData;
    bool m_isWrite;
    uint8_t m_portgVal;

    QHash<uint16_t, uint8_t> m_externalRamSimulation;
};

#endif // MCUXMEMPROXY_H