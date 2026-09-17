/***************************************************************************
 *   Copyright (C) 2020 by Santiago González                               *
 *                                                                         *
 ***( see copyright.txt file at root folder )*******************************/
/*
 *   Based on simavr decoder
 *
 *   Copyright 2008, 2010 Michel Pollet <buserror@gmail.com>
 *
 */

#pragma once

#include "mcu8bits.h"
#include "mcutypes.h"
#include "rammemoryproxy.h"

class AvrCore : public Mcu8bits {
public:
    AvrCore( eMcu* mcu );
    ~AvrCore();

    virtual void reset() override;
    virtual void runStep() override;

    void setRamProxy( RamMemoryProxy* proxy );
    RamMemoryProxy* ramProxy() const { return m_ramProxy; }

    inline uint8_t readDataMem( uint16_t addr );
    inline void    writeDataMem( uint16_t addr, uint8_t val );

private:
    void writeFlash();

    uint16_t m_bootStart;

    int m_pageSize;
    std::vector<uint8_t> m_tmpUsed;
    std::vector<uint16_t> m_tmpPage;

    regBits_t m_SELFPRGEN;
    regBits_t m_PGERS;
    regBits_t m_PGWRT;

    uint16_t m_rampzAddr;
    uint8_t* RAMPZ; // optional, only for ELPM/SPM on >64Kb cores
    uint8_t* EIND; // optional, only for EIJMP/EICALL on >64Kb cores

    RamMemoryProxy* m_ramProxy;
    DirectRamProxy  m_defaultProxy;

    void flags_ns( uint8_t res );
    void flags_zns( uint8_t res );
    void flags_Rzns( uint8_t res );
    void flags_sub( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_sub_Rzns( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_add_zns( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_sub_zns( uint8_t res, uint8_t rd, uint8_t rr );
    void flags_znv0s( uint8_t res );
    void flags_zcnvs( uint8_t res, uint8_t vr );
    void flags_zcvs( uint8_t res, uint8_t vr );
    void flags_zns16( uint16_t res );
    int is_instr_32b( uint32_t pc );
};

inline uint8_t AvrCore::readDataMem( uint16_t addr )
{
    return m_ramProxy->read( addr );
}

inline void AvrCore::writeDataMem( uint16_t addr, uint8_t val )
{
    m_ramProxy->write( addr, val );
}

