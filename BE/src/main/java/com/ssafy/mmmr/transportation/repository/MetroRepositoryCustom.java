package com.ssafy.mmmr.transportation.repository;

import java.util.List;

import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.transportation.entity.MetroEntity;

public interface MetroRepositoryCustom {
	List<MetroEntity> findActiveMetrosByProfile(ProfileEntity profile);

	int countActiveMetrosByProfile(ProfileEntity profile);
}