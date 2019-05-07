/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2018,  Regents of the University of California,
 *                           Arizona Board of Regents,
 *                           Colorado State University,
 *                           University Pierre & Marie Curie, Sorbonne University,
 *                           Washington University in St. Louis,
 *                           Beijing Institute of Technology,
 *                           The University of Memphis.
 *
 * This file is part of NFD (Named Data Networking Forwarding Daemon).
 * See AUTHORS.md for complete list of NFD authors and contributors.
 *
 * NFD is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NFD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NFD, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "clf-vanet-strategy.hpp"
#include "algorithm.hpp"
#include "core/logger.hpp"

#include <ndn-cxx/lp/tags.hpp>

#include "ns3/simulator.h"
#include "ns3/nstime.h"

namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(ClfStrategy);

NFD_LOG_INIT(ClfStrategy);

const double ClfStrategy::ALPHA = 0.5;

ClfStrategy::ClfStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  //, m_scheduler(m_ioService)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    BOOST_THROW_EXCEPTION(std::invalid_argument("ClfStrategy does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    BOOST_THROW_EXCEPTION(std::invalid_argument(
      "ClfStrategy does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));
}

const Name&
ClfStrategy::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/clf/%FD%03");
  return strategyName;
}

void
ClfStrategy::afterReceiveInterest(const Face& inFace, const Interest& interest,
                                  const shared_ptr<pit::Entry>& pitEntry)
{
  //afterReceiveInterestBroadcast(inFace, interest, pitEntry);

  afterReceiveInterestLocation(inFace, interest, pitEntry);
  //afterReceiveInterestLocationCentrality(inFace, interest, pitEntry);
}

void
ClfStrategy::afterReceiveInterestBroadcast(const Face& inFace, const Interest& interest,
                                           const shared_ptr<pit::Entry>& pitEntry)
{
  // get forwarding information
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();
  
  // forward interest
  bool isProducer = false; 
  for (const auto& nexthop : nexthops) {
    Face& outFace = nexthop.getFace();
    if (outFace.getLinkType() == ndn::nfd::LINK_TYPE_POINT_TO_POINT) {
      isProducer = true; 
    }
  }
    
  for (const auto& nexthop : nexthops) {
    Face& outFace = nexthop.getFace();
    if (isProducer && outFace.getLinkType() == ndn::nfd::LINK_TYPE_POINT_TO_POINT) {
      NFD_LOG_DEBUG(interest << " from=" << inFace.getId()
                         << " pitEntry-to=" << outFace.getId() << ", outface link type: " << outFace.getLinkType());
      this->sendInterest(pitEntry, outFace, interest);
      break;
    }
    
    if (!isProducer && outFace.getLinkType() == ndn::nfd::LINK_TYPE_AD_HOC) {
      NFD_LOG_DEBUG(interest << " from=" << inFace.getId()
                         << " pitEntry-to=" << outFace.getId() << ", outface link type: " << outFace.getLinkType());
      this->sendInterest(pitEntry, outFace, interest);
      break;
    }
  }
}

void
ClfStrategy::forwardInterest(const Interest& interest,
                             const shared_ptr<pit::Entry>& pitEntry,
                             //TODO: FaceInfo* info,
                             Face& outFace)
{
  //std::cout << "Forwarding scheduled interest: " << interest;
  NFD_LOG_DEBUG("Forwarding scheduled interest: " << interest);
  // send the interest
  this->sendInterest(pitEntry, outFace, interest);
  
  // remove the interest from the pool, because it has been sent.
  //m_scheduledInterstPool.erase(interest.getName());

  // TODO: update the score of this prefix
  //info->incrementInterest();
}

void
ClfStrategy::afterReceiveInterestLocation(const Face& inFace, const Interest& interest,
                                          const shared_ptr<pit::Entry>& pitEntry)
{
  // get location information
  auto locationTag = interest.getTag<lp::LocationTag>();
  ndn::Location ml;
  ndn::Location pl;
  ndn::Location dl;
  if (locationTag != nullptr) {
    ml = locationTag->get().getMyLocation();
    pl = locationTag->get().getPrevLocation();
    dl = locationTag->get().getDestLocation();
    NFD_LOG_DEBUG("MyLocation: " << ml.getLatitude() << ", " << ml.getLongitude());
    NFD_LOG_DEBUG("PrevLocation: " << pl.getLatitude() << ", " << pl.getLongitude());
    NFD_LOG_DEBUG("DestLocation: " << dl.getLatitude() << ", " << dl.getLongitude());
  }
  else {
    NFD_LOG_DEBUG("LocationHeader not found in the interest."); 
  }

  // calculate weight based on location and centrlaity score (TODO) 
  double weight = 0;
  double centralityScore = getCentralityScore(interest.getName());
  if (dl.getLongitude() != 0 or dl.getLatitude() != 0) {// if interest contains data location field
    double locationScore = getLocationScore(pl, dl, ml);
    NFD_LOG_DEBUG("Location score: " << locationScore);
    weight = ALPHA * (1 - locationScore) + (1-ALPHA) * centralityScore; 
  }
  else {
    /* if prefix location table has destination location
         double locationScore = getLocationScore(pl, dl, ml);
         double weight = ALPHA * (1 - locationScore) + (1-ALPHA) * centralityScore;
       else
         weight = (1-ALPHA) * centralityScore;;
     **/
  }
 
  NFD_LOG_DEBUG("Weight = " << weight);

  // calcualte waiting time based on weight
  time::milliseconds waitingTime(0);
  /*if (weight != 0) {
    waitingTime = static_cast<time::milliseconds::rep>(1/weight);
    NFD_LOG_DEBUG(interest << " scheduler after " << waitingTime);
  }
  else {
    waitingTime = 0;
  }*/

  // get forwarding information
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();
     
  // forward interest
  bool isProducer = false; 
  for (const auto& nexthop : nexthops) {
    Face& outFace = nexthop.getFace();
    if (outFace.getLinkType() == ndn::nfd::LINK_TYPE_POINT_TO_POINT) {
      isProducer = true; 
    }
  }
   
  // forward interest 
  for (const auto& nexthop : nexthops) {
    Face& outFace = nexthop.getFace();
     if (isProducer && outFace.getLinkType() == ndn::nfd::LINK_TYPE_POINT_TO_POINT) {
       NFD_LOG_DEBUG(interest << " from=" << inFace.getId()
                         << " pitEntry-to=" << outFace.getId() << ", outface link type: " << outFace.getLinkType());
 
       this->sendInterest(pitEntry, outFace, interest);
       break;
     }
     
     if (!isProducer && outFace.getLinkType() == ndn::nfd::LINK_TYPE_AD_HOC) {
       NFD_LOG_DEBUG(interest << " from=" << inFace.getId()
                         << " pitEntry-to=" << outFace.getId() << ", outface link type: " << outFace.getLinkType() << ", scheduled after " << waitingTime);

      ndn::EventId eventId = scheduler::schedule(waitingTime,
                                [this, &interest, pitEntry, &outFace] {forwardInterest(interest, pitEntry,
                                                                   /*info,*/ outFace);});
      break;
      //::ns3::Simulator::Schedule(::ns3::Seconds(1), &ClfStrategy::forwardInterest, this, interest, pitEntry/*, outFace*/);
    }
  }
}

void
ClfStrategy::afterReceiveNack(const Face& inFace, const lp::Nack& nack,
                                    const shared_ptr<pit::Entry>& pitEntry)
{
  //this->processNack(inFace, nack, pitEntry);
}

double
ClfStrategy::calculateDistance(double lat1d, double lon1d,
                                 double lat2d, double lon2d)
{
  double lat1r, lon1r, lat2r, lon2r, u, v;

  lat1r = deg2rad(lat1d);
  lon1r = deg2rad(lon1d);
  lat2r = deg2rad(lat2d);
  lon2r = deg2rad(lon2d);

  u = sin((lat2r - lat1r)/2);
  v = sin((lon2r - lon1r)/2);

  return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

} // namespace fw
} // namespace nfd
