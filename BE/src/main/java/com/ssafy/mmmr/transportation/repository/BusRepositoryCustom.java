package com.ssafy.mmmr.transportation.repository;

import java.util.List;

import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.transportation.entity.BusEntity;

public interface BusRepositoryCustom {
	List<BusEntity> findActiveBusesByProfile(ProfileEntity profile);

	int countActiveBusesByProfile(ProfileEntity profile);
}