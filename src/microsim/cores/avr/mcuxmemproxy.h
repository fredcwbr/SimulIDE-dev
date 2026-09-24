#ifndef MCUXMEMPROXY_H
#define MCUXMEMPROXY_H

#include "rammemoryproxy.h"
#include "avrcore.h"
#include "e_mcu.h"
#include "mcuport.h"
#include "mcupin.h"
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
        , m_dataPort(nullptr)
        , portC(nullptr)
        , portA(nullptr)
        , portG(nullptr)
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
        , m_nWaits(0)
    {
        qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] Instantiate  ph 1 |";
        
        if ( m_mcu ) {
            qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] Instantiate 2 ph m_mcu |";
            portC = m_mcu->getMcuPort("PORTC");
            if (portC) {
                portC->controlPort( true, true );
            }
            
            portA = m_mcu->getMcuPort("PORTA");
            if (portA) {
                portA->controlPort( true, true );
                portA->setDirection( 0xFF );  // all output high address byte
            }
            
            //void setDirection( uint val );       // Direct control over pins
            //void setOutState( uint val );        // Direct control over pins
            // uint getInpState();             // Direct control over pins
            portG = m_mcu->getMcuPort("PORTG");
            if (portG) {
                portG->controlPort( true, true );
                portG->setDirection( 0x07 );       // Permanente para toda simulacao ., ...
            }

            qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] Instantiate  ph 3 m_mcu  |" << "\n"
                    << "| m_mcu  " << m_mcu << "\n"
                    << "| m_avrCore  " << m_avrCore << "\n"
                    << "| portA " << Qt::hex << portA << "\n"
                    << "| portC " << Qt::hex << portC << "\n"
                    << "| portG " << Qt::hex << portG << "\n"
                    ;
          

            uint64_t basePsInst = 62500; // Fallback for 16 MHz
            _1ns = 1000;

            uint64_t cycleTime = basePsInst * 4; 
            m_psStep       = cycleTime / 12;
            m_addrSetTime  = 3 * m_psStep;
            m_laEnEndTime  = 4 * m_psStep;
            m_readSetTime  = 6 * m_psStep;
            m_writeSetTime = 6 * m_psStep;
            m_readBusTime  = (cycleTime > 10) ? (cycleTime - 10) : 0;

            qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM-TIMING-INIT] AVR Core (4 clocks/inst) | BasePsInst:" << basePsInst
                     << "CycleTime:" << cycleTime
                     / 1000  << "nsStep:" << m_psStep
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
            qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM-PROXY] XMCRA configured with:" << Qt::hex << value;
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
            qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM-PROXY] External XMEM Write at address:" << Qt::hex << address << "Data:" << value;
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
        m_nWaits = 0;
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

            qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM-TIMING-RESET] CycleTime(psInst):" << cycleTime
                     / 1000  << "nsStep:" << m_psStep
                     << "addrSetTime:" << m_addrSetTime
                     << "readSetTime:" << m_readSetTime
                     << "writeSetTime:" << m_writeSetTime;
        }
    }

    void runEvent() 
    {
        qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] runEvent start. xmemState  " << m_xmemState;
        switch (m_xmemState)
        {
            case xmem_IDLE:
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: IDLE - Waiting for new cycle.";
                break;

            case xmem_CYCLE_START:
            {
                                       
                uint8_t lowAddr = static_cast<uint8_t>(m_targetAddress & 0xFF);
                uint8_t highAddr = static_cast<uint8_t>((m_targetAddress >> 8) & 0xFF);
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: CYCLE_START | ph 1"
                << "| portC  " << Qt::hex << portC  
                ;

                // only portC is multiplexed.,     
                portC->setDirection( 0xFF );       // Direct control over pins
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: CYCLE_START | ph 3"
                << "| portA  " << Qt::hex << portA  
                ;
                portA->outChanged( highAddr );
                
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: CYCLE_START | ph 4"
                << "| portC  " << Qt::hex << portC  
                ;
                portC->outChanged( lowAddr);
                                
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: CYCLE_START"
                         << "| Addr:" << Qt::hex << m_targetAddress
                         << "| Dir:" << (m_isWrite ? "WRITE" : "READ")
                         << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);

                Simulator::self()->addEvent(_1ns, this);
                m_xmemState = xmem_ALE_HIGH;
            }
            break;

            case xmem_ALE_HIGH:
            {
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: ALE_HIGH | ph 1"
                << "| portG  " << Qt::hex << portG  << " | portGValue: " << Qt::hex << portG->getInpState()
                ;
                portG->outChanged(0x07); // Set ALE high bit
                
                qDebug() << "[Time:" << Simulator::self()->circTime()/ 1000  << "ns] [XMEM] State: ALE_HIGH"
                         << "| Addr:" << Qt::hex << m_targetAddress
                         << "| Dir:" << (m_isWrite ? "WRITE" : "READ")
                         << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);

                Simulator::self()->addEvent(_1ns, this);
                m_xmemState = xmem_ALE_HOLD;
            }
            break;

            case xmem_ALE_HOLD:
            {
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: ALE_HOLD"
                         << "| Addr:" << Qt::hex << m_targetAddress
                         << "| Dir:" << (m_isWrite ? "WRITE" : "READ")
                         << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);


                Simulator::self()->addEvent(m_psStep, this);
                m_xmemState = xmem_ALE_LOW;
            }
            break;

            case xmem_ALE_LOW:
            {
                
                portG->getPinN(2)->setOutState(0); // Set ALE high bit
                
                if (m_isWrite) {
                    if (portC && m_targetData) {
                     portC->setDirection( 0xFF );       // Direct control over pins 
                     portC->outChanged( *m_targetData );

                    qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: ALE_LOW (WRITE)"
                             << "| Addr:" << Qt::hex << m_targetAddress
                             << "| Dir: WRITE"
                             << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);

                    Simulator::self()->addEvent(_1ns, this);
                    m_xmemState = xmem_STROBE_ACTIVE;
                } else {

                     portC->setDirection( 0x00 );       // Direct control over pins INPUT
                     //void setOutState( uint val );    // Direct control over pins
                     // portC->setOutState( 0 );        // this should not be necessary,    
                    
                     
                    }

                    qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: ALE_LOW (READ)"
                             << "| Addr:" << Qt::hex << m_targetAddress
                             << "| Dir: READ"
                             << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);


                    Simulator::self()->addEvent(_1ns, this);
                    m_xmemState = xmem_STROBE_ACTIVE;
                }
            }
            break;

            case xmem_STROBE_ACTIVE:
            {
                m_nWaits = 1;       // at least 1 ..  more on wait flags, of XMCRA
                
                if (m_isWrite) {
                    portG->outChanged(0x02); // Set WR active low
                    
                    qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: STROBE_ACTIVE (WRITE)"
                             << "| Addr:" << Qt::hex << m_targetAddress
                             << "| Dir: WRITE"
                             << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);

                } else {
                    portG->outChanged(0x01); // Set RD active low

                    qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: STROBE_ACTIVE (READ)"
                             << "| Addr:" << Qt::hex << m_targetAddress
                             << "| Dir: READ"
                             << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);

               }

               Simulator::self()->addEvent(_1ns, this);
               m_xmemState = xmem_STROBE_WAIT;
            }
            break;

            case xmem_STROBE_WAIT:
            {
                if (m_nWaits > 0) {
                    m_nWaits--;
                    qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  
                             << "ns] [XMEM] State: STROBE_WAIT (Decrementing wait states, remaining:" 
                             << m_nWaits << ")"
                             << "| Addr:" << Qt::hex << m_targetAddress
                             << "| Dir:" << (m_isWrite ? "WRITE" : "READ")
                             << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);


                    Simulator::self()->addEvent(4 * m_psStep, this);
                    m_xmemState = xmem_STROBE_WAIT;
                } else {
                    qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: STROBE_WAIT (Wait states finished, moving to STROBE_END)"
                             << "| Addr:" << Qt::hex << m_targetAddress
                             << "| Dir:" << (m_isWrite ? "WRITE" : "READ")
                             << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);


                    Simulator::self()->addEvent(m_psStep, this);
                    m_xmemState = xmem_STROBE_END;
                }
            }
            break;

            case xmem_STROBE_END:
            {
                if (!m_isWrite) {
                    if (m_targetData) {
                        *m_targetData =  portC->getInpState();             // Direct control over pins
                    } 
                }

                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: STROBE_END"
                         << "| Addr:" << Qt::hex << m_targetAddress
                         << "| Dir:" << (m_isWrite ? "WRITE" : "READ")
                         << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);


                Simulator::self()->addEvent(_1ns, this);
                m_xmemState = xmem_STROBE_DEASSERT;
            }
            break;

            case xmem_STROBE_DEASSERT:
            {
                portG->outChanged(0x03);
                
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] State: STROBE_DEASSERT (Cycle Complete)"
                         << "| Addr:" << Qt::hex << m_targetAddress
                         << "| Dir:" << (m_isWrite ? "WRITE" : "READ")
                         << "| DataVal:" << (m_targetData ? *m_targetData : 0)
                         << "| ALE:" << (portG ? portG->getPinN(2)->getOutState() : 0)
                         << "| RD:" << (portG ? portG->getPinN(1)->getOutState() : 0)
                         << "| WR:" << (portG ? portG->getPinN(0)->getOutState() : 0);


                Simulator::self()->addEvent( _1ns, this);
                m_xmemState = xmem_IDLE;
            
                break;

            default:
                qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] ERROR: Unknown state in state machine. Resetting to IDLE.";
                m_xmemState = xmem_IDLE;
                break;
        }
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
            qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM-REG] XMCRA: SRE=" << m_sre << "SRW0=" << m_srw0 << "SRW1=" << m_srw1;
        }
    }

    void triggerBusCycle(uint16_t address, uint8_t& data, bool isWrite) {
        if (!m_mcu || !m_avrCore) return;

        qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM] trigger entry  |" << "\n"
                    << "| m_mcu  " << m_mcu << "\n"
                    << "| m_avrCore  " << m_avrCore << "\n"
                    << "| portA " << Qt::hex << portA << "\n"
                    << "| portC " << Qt::hex << portC << "\n"
                    ;
            
        qDebug() << "[Time:" << Simulator::self()->circTime() / 1000  << "ns] [XMEM-DEBUG] Triggering Bus Cycle for Addr:" << Qt::hex << address;

        m_targetAddress = address;
        m_targetData = &data;
        m_isWrite = isWrite;

        m_xmemState = xmem_CYCLE_START;
        Simulator::self()->addEvent(1, this);
    }

    enum xmemState_t {
        xmem_IDLE = 0,
        xmem_CYCLE_START,
        xmem_ALE_HIGH,
        xmem_ALE_HOLD,
        xmem_ALE_LOW,
        xmem_STROBE_ACTIVE,
        xmem_STROBE_WAIT,
        xmem_STROBE_END,
        xmem_STROBE_DEASSERT
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

    McuPort* m_dataPort;
    McuPort* portC;
    McuPort* portA;
    McuPort* portG;

    uint64_t m_psStep;
    uint64_t m_addrSetTime;
    uint64_t m_laEnEndTime;
    uint64_t m_readSetTime;
    uint64_t m_writeSetTime;
    uint64_t m_readBusTime;
    uint64_t m_dataTime;
    uint64_t _1ns;

    xmemState_t m_xmemState;
    uint16_t m_targetAddress;
    uint8_t* m_targetData;
    bool m_isWrite;
    int m_nWaits;

    QHash<uint16_t, uint8_t> m_externalRamSimulation;
};

#endif // MCUXMEMPROXY_H