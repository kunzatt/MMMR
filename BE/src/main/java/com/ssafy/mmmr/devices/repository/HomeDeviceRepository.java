package com.ssafy.mmmr.devices.repository;

import java.util.List;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.devices.entity.HomeDeviceEntity;
import com.ssafy.mmmr.account.entity.AccountEntity;

public interface HomeDeviceRepository extends JpaRepository<HomeDeviceEntity, Long> {
	List<HomeDeviceEntity> findByAccount(AccountEntity account);
}