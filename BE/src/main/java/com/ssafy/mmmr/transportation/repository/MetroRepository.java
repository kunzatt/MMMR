package com.ssafy.mmmr.transportation.repository;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.transportation.entity.MetroEntity;

public interface MetroRepository extends JpaRepository<MetroEntity, Long> {
}